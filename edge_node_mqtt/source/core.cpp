/**
 * @file core.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "mqtt_maeve.hpp"

#include <atomic>
#include <chrono>
#include <thread>
#include <signal.h>


namespace maeve {

std::atomic<bool> g_exit_flag(false);
static void on_exit(int signum)
{
    g_exit_flag = true;
}

MqttMaeve::MqttMaeve(Maeve& platform, fs::path conf_fp, fs::path out_conf_fp,
    std::int64_t skip_frames_until_time, std::string wait_start_until_time)
    : _platform{platform}
    , _conf_fp(
        conf_fp.empty() ? fs::path(Utils::get_host_name() + ".ini") : conf_fp)
    , _mqtt_conf(parser::Config(_conf_fp)["MQTT"])
    , _calib_conf(parser::Config(_conf_fp)["CALIBRATION"])
    , _rtsp_conf(parser::Config(_conf_fp)["RTSP"])
    , _cli{_mqtt_conf.uri, _mqtt_conf.client_id}
    , _out_conf_fp{out_conf_fp}
    , _human_parts{HUMAN_PARTS}
    , _skip_frames_until_time{skip_frames_until_time}
    , _wait_start_until_time{wait_start_until_time}
    , _init_times{}
    , _pub_times{}
    , _loop_times{}
    , _cam_times{}
    , _exec_times{}
    , _received_amount{}
    , _matrix_analysis({
        { "publish_duration", _pub_times.values()  },
        { "loop_duration"   , _loop_times.values() },
        { "cam_duration"    , _cam_times.values()  },
        { "exec_duration"   , _exec_times.values() },
        { "received_frames" , _received_amount     }  
        })
{
    // Prepare human parts.
    for (const auto& hp: HP_TO_REMOVE)
    {
        _human_parts.erase(std::remove(
            _human_parts.begin(), _human_parts.end(), hp), _human_parts.end());
    }
    for (const auto& hp: HP_TO_INSERT)
    {
        _human_parts.push_back(hp);
    }

    // Prepare human parts idx.
    _shoulders_idx = Math::idx(_human_parts, HP_SHOULDERS);
    _hips_idx = Math::idx(_human_parts, HP_HIPS);
    _mid_shoulder_idx = Math::contains(_shoulders_idx, std::int64_t(-1)) ?
        -1 : Math::idx(_human_parts, HP_MID_SHOULDERS);
    _mid_hip_idx = Math::contains(_hips_idx, std::int64_t(-1)) ? 
        -1 : Math::idx(_human_parts, HP_MID_HIPS);
}

MqttMaeve::~MqttMaeve()
{
    _matrix_analysis.dump_values(fs::path("mqttmaeve_time_analysis"), 0);
    _matrix_analysis.dump_analysis(fs::path("mqttmaeve_time_analysis"), 5);

    try
    {
        _cli.disconnect()->wait();
    }
    catch (const mqtt::exception& exc) 
    {
        std::cerr << exc << std::endl;
    }
}

void MqttMaeve::run_kp3d(bool exit_on_stream_failure)
{
    std::size_t iter = 0;
    auto first_input = _init();

    if (_mqtt_conf.topic.empty())
    {
        _mqtt_conf.topic = DFLT_ROOT_TOPIC + "/kp3d/"
            + Utils::get_host_name() + "_" +  Utils::get_machine_id();
    }
    std::string topic = _mqtt_conf.topic;

    // Convert device configuration frequency in period. 
    using fsec = std::chrono::duration<double>;
    auto period = fsec(1.0 / _platform.device().fps());

    try
    {
        mqtt::topic top(_cli, topic, _mqtt_conf.qos);

        auto color_frame = std::get<0>(first_input);
        auto depth_frame = std::get<1>(first_input);
        auto frame_id    = _platform.device().frame_id();
        if (!color_frame.empty() && !depth_frame.empty())
        {
            json obj;
            auto exit_flag = _exec(obj, first_input, frame_id, iter++);
            if (exit_flag) return;
            _pub_times.start();
            auto tok = top.publish(obj.dump());
            tok->wait();
            _pub_times.stop();
        }

        while (!g_exit_flag)
        {
            _loop_times.start();
            _cam_times.start();
            auto prev_get_frame = std::chrono::steady_clock::now();
            auto frameset = _platform.get_frame();
            auto frame_id = _platform.device().frame_id();
            _cam_times.stop();

            json obj;
            auto exit_flag = _exec(obj, frameset, frame_id, iter, 
                                   exit_on_stream_failure);
            if (exit_flag) break;

            _pub_times.start();
            auto tok = top.publish(obj.dump());
            tok->wait();
            _pub_times.stop();

            if (_platform.device().is_file_enabled())
            {
                auto time_for_fps = prev_get_frame + period;
                std::this_thread::sleep_until(time_for_fps);
            }
            _loop_times.stop();

            _print_loop_stats(frame_id, iter);
            _matrix_analysis.dump_values(
                fs::path("mqttmaeve_time_analysis"), 0);
            ++iter;
        }
    }
    catch (const mqtt::exception& exc) 
    {
        throw std::runtime_error("MQTT exception: " + exc.get_message());
    }

    std::cout 
        << "[" << Utils::timestamp_conv() <<  "] { " 
        << "received_frames: " << iter << "; "
        << " }"<< std::endl;
    _received_amount.push_back(iter);
}

void MqttMaeve::run_kp3d_dump(bool exit_on_stream_failure)
{
    _init_times.start();

    // Set camera parameters.
    if (!_calib_conf.extrinsics_fp.empty())
    {
        _platform.device().set_extrinsic_params(
            _calib_conf.extrinsics_fp, _calib_conf.separator);
    }

    if (!_calib_conf.intrinsics_fp.empty())
    {
        _platform.device().set_intrinsic_params(
            _calib_conf.intrinsics_fp, _calib_conf.separator);
    }

    // Print params.
    std::cout << "Application config: " << std::endl;
    _mqtt_conf.print();
    _calib_conf.print();
    _rtsp_conf.print();

    // Store params.
    if (!_out_conf_fp.empty())
    {
        _mqtt_conf.store(_out_conf_fp);
        _calib_conf.store(_out_conf_fp);
        _rtsp_conf.store(_out_conf_fp);
    }

    std::vector<json> obj_vec;
    std::size_t iter = 0;

    // Skip frames until a timestamp (int64 - nanoseconds).
    auto first_input = _skip_frames_until(_skip_frames_until_time);

    // Wait until time. 
    if (!_wait_start_until_time.empty())
    {
        auto wait_until = Utils::timestamp_conv(_wait_start_until_time);
        _wait_start_until(wait_until);
    }

    // Init platform.
    _platform.init();
    _init_times.stop();

    auto frame_id    = _platform.device().frame_id();
    auto color_frame = std::get<0>(first_input);
    auto depth_frame = std::get<1>(first_input);
    if (!color_frame.empty() && !depth_frame.empty())
    {
        json obj;
        auto exit_flag = _exec(obj, first_input, frame_id, iter++);
        if (exit_flag) return;
        obj_vec.push_back(obj);
    }

    signal(SIGINT, on_exit);
    while (!g_exit_flag)
    {
        _loop_times.start();
        _cam_times.start();
        auto prev_get_frame = std::chrono::steady_clock::now();
        auto frameset = _platform.get_frame();
        auto frame_id = _platform.device().frame_id();
        _cam_times.stop();
        
        json obj;
        auto exit_flag = _exec(obj, frameset, frame_id, iter, 
                               exit_on_stream_failure);
        if (exit_flag) break;
        if (obj.empty())
        {
            _loop_times.stop();
            continue;
        }
        obj_vec.push_back(obj);

        _loop_times.stop();
        _print_loop_stats(frame_id, iter);
        _matrix_analysis.dump_values(
            fs::path("mqttmaeve_time_analysis"), 0);
        ++iter;
    }

    std::ofstream o("data.json");
    o << std::setw(4) << json(obj_vec) << std::endl;

    std::cout 
        << "[" << Utils::timestamp_conv() <<  "] { " 
        << "received_frames: " << iter << "; "
        << " }"<< std::endl;
    _received_amount.push_back(iter);
}

void MqttMaeve::run_kp3d_stream_only(bool exit_on_stream_failure)
{
    _init_times.start();

    // Set camera parameters.
    if (!_calib_conf.extrinsics_fp.empty())
    {
        _platform.device().set_extrinsic_params(
            _calib_conf.extrinsics_fp, _calib_conf.separator);
    }

    if (!_calib_conf.intrinsics_fp.empty())
    {
        _platform.device().set_intrinsic_params(
            _calib_conf.intrinsics_fp, _calib_conf.separator);
    }

    // Print params.
    std::cout << "Application config: " << std::endl;
    _mqtt_conf.print();
    _calib_conf.print();
    _rtsp_conf.print();

    // Store params.
    if (!_out_conf_fp.empty())
    {
        _mqtt_conf.store(_out_conf_fp);
        _calib_conf.store(_out_conf_fp);
        _rtsp_conf.store(_out_conf_fp);
    }

    std::size_t iter = 0;

    // Skip frames until a timestamp (int64 - nanoseconds).
    auto first_input = _skip_frames_until(_skip_frames_until_time);

    // Wait until time. 
    if (!_wait_start_until_time.empty())
    {
        auto wait_until = Utils::timestamp_conv(_wait_start_until_time);
        _wait_start_until(wait_until);
    }

    // Init platform.
    _platform.init();
    _init_times.stop();

    while (!g_exit_flag)
    {
        _loop_times.start();
        _cam_times.start();
        auto prev_get_frame = std::chrono::steady_clock::now();
        auto frameset = _platform.get_frame();
        auto frame_id = _platform.device().frame_id();
        _cam_times.stop();
        _loop_times.stop();
        _print_loop_stats(frame_id, iter);
        _matrix_analysis.dump_values(
            fs::path("mqttmaeve_time_analysis"), 0);
        ++iter;
    }

    std::cout 
        << "[" << Utils::timestamp_conv() <<  "] { " 
        << "received_frames: " << iter << "; "
        << " }"<< std::endl;
    _received_amount.push_back(iter);
}

void MqttMaeve::run_kp3d_frame(std::uint64_t tframe)
{
    _init();
    if (_mqtt_conf.topic.empty())
    {
        _mqtt_conf.topic = DFLT_ROOT_TOPIC + "/kp3d/"
            + Utils::get_host_name() + "_" +  Utils::get_machine_id();
    }
    std::string topic = _mqtt_conf.topic;

    std::cout << "Start to wait until " << Utils::timestamp_conv(tframe * 1e-9) 
        << std::endl;

    auto frameset = _skip_frames_until(tframe);
    auto frame_id = _platform.device().frame_id();
    auto t = std::get<2>(frameset); 
    
    json obj;
    auto exit_flag = _exec(obj, frameset, frame_id, 0);
    if (exit_flag) return;
    _received_amount.push_back(1);

    try
    {
        mqtt::topic top(_cli, topic, _mqtt_conf.qos);
        mqtt::token_ptr tok;
        while (!g_exit_flag)
        {
            _loop_times.start();
            _pub_times.start();
            auto timestamp_pub = std::uint64_t(
                std::chrono::high_resolution_clock::now()
                    .time_since_epoch().count());
            obj["timestamp_pub"] = double(timestamp_pub) * 1e-6; //< ms
            tok = top.publish(obj.dump());
            tok->wait();
            _pub_times.stop();
            _loop_times.stop();

            std::cout 
                << "[" << Utils::timestamp_conv() <<  "] { " 
                << "t_frame: " << t << " ns; "
                << "t_pub: " << timestamp_pub << " ns; " 
                << "loop_time: " << _loop_times.elapsed() << " ms; "
                << "pub_time: " << _pub_times.elapsed() << " ms; "
                << "loop_fps: " 
                    <<  (1.0 / _loop_times.elapsed()) * 1000.0 << " hz; "
                << " }"<< std::endl;
            _matrix_analysis.dump_values(
                fs::path("mqttmaeve_time_analysis"), 0);
        }
    }
    catch (const mqtt::exception& exc) 
    {
        throw std::runtime_error("MQTT exception: " + exc.get_message());
    }
}

void MqttMaeve::run_kp3d_rtsp()
{
    _init();
    if (_mqtt_conf.topic.empty())
    {
        _mqtt_conf.topic = DFLT_ROOT_TOPIC + "/kp3d/"
            + Utils::get_host_name() + "_" +  Utils::get_machine_id();
    }
    std::string topic = _mqtt_conf.topic;

    // Init RTSP pipeline.
    if (_rtsp_conf.location.empty())
    {
        throw std::runtime_error("RTSP configuration error: empty location");
    }
    const std::string pipeline_options 
        = "latency=0 ! decodebin ! autovideoconvert ! appsink";
    const std::string pipeline 
        = "rtspsrc location=" + _rtsp_conf.location + " " + pipeline_options;
    
    cv::VideoCapture v(pipeline);

    if (!v.isOpened())
    {
        throw std::runtime_error("Cannot open cv::VideoCapture RTSP pipeline");
    }

    std::size_t iter = 0;
    try
    {
        mqtt::topic top(_cli, topic, _mqtt_conf.qos);
        mqtt::token_ptr tok;
        cv::Mat frameset;
        while (!g_exit_flag)
        {
            _loop_times.start();
            _cam_times.start();
            v >> frameset;
            if (frameset.empty()) continue;
            _cam_times.stop();
            _exec_times.start();
            auto color_frame = cv::Mat(frameset.rows / 2, frameset.cols, 
                CV_8UC3, frameset.ptr<std::uint8_t>(0),
                frameset.step);
            auto depth_frame = cv::Mat(frameset.rows / 2, frameset.cols, 
                CV_32FC1, frameset.ptr<std::uint8_t>(frameset.rows / 2), 
                frameset.step);
            auto timestamp = v.get(cv::CAP_PROP_POS_MSEC); 

            json obj;
            auto exit_flag = _exec(obj, {color_frame, depth_frame, timestamp}, 
                                   iter, iter);
            if (exit_flag) return;

            _pub_times.start();
            tok = top.publish(obj.dump());
            tok->wait();
            _pub_times.stop();

            _loop_times.stop();
            _print_loop_stats(iter, iter);
            _matrix_analysis.dump_values(
                fs::path("mqttmaeve_time_analysis"), 0);
            ++iter;
        }
    }
    catch (const mqtt::exception& exc) 
    {
        throw std::runtime_error("MQTT exception: " + exc.get_message());
    }
    _received_amount.push_back(iter);
}

std::tuple<cv::Mat, cv::Mat, std::uint64_t> MqttMaeve::_init()
{
    _init_times.start();

    mqtt::connect_options conn_options;
    if (!_mqtt_conf.username.empty())
    {
        conn_options = mqtt::connect_options_builder()
            .user_name(_mqtt_conf.username)
            .password(_mqtt_conf.password)
            .finalize();
    }

    // Set camera parameters.
    if (!_calib_conf.extrinsics_fp.empty())
    {
        _platform.device().set_extrinsic_params(
            _calib_conf.extrinsics_fp, _calib_conf.separator);
    }

    if (!_calib_conf.intrinsics_fp.empty())
    {
        _platform.device().set_intrinsic_params(
            _calib_conf.intrinsics_fp, _calib_conf.separator);
    }

    // Print params.
    std::cout << "Application config: " << std::endl;
    _mqtt_conf.print();
    _calib_conf.print();
    _rtsp_conf.print();

    // Store params.
    if (!_out_conf_fp.empty())
    {
        _mqtt_conf.store(_out_conf_fp);
        _calib_conf.store(_out_conf_fp);
        _rtsp_conf.store(_out_conf_fp);
    }

    // Skip frames until a timestamp (int64 - nanoseconds).
    auto first_input = _skip_frames_until(_skip_frames_until_time);

    // Wait until time. 
    if (!_wait_start_until_time.empty())
    {
        auto wait_until = Utils::timestamp_conv(_wait_start_until_time);
        _wait_start_until(wait_until);
    }

    // Init platform.
    _platform.init();

    try
    {
        _cli.connect(conn_options)->wait();
        _init_times.stop();
        std::cout << "[" << Utils::timestamp_conv() << "] { " 
            << "init_time: " << _init_times.elapsed() << " ms"
            << " }" << std::endl; 
        analysis::Vector<double>(_init_times.values(), "init_duration")
            .dump_values(fs::path("mqttmaeve_time_analysis"), 0);
    }
    catch (const mqtt::exception& exc) 
    {
        throw std::runtime_error("MQTT exception: " + exc.get_message());
    }

    // Init signal exit on SIGINT. 
    signal(SIGINT, on_exit);

    return first_input;
}

MqttMaeve::BeFineMultiHumanKeypoints<float> MqttMaeve::_postprocessing(
    const MultiHumanKeypoints<float>& kp3d, 
    const std::vector<std::string>& human_parts)
{
    MqttMaeve::BeFineMultiHumanKeypoints<float> data;
    for (std::size_t h_idx = 0; h_idx < kp3d.size(); ++h_idx)
    {
        auto h = kp3d[h_idx];
        if (h.size() <= KP_AMOUNT_TO_EXCLUDE)
        {
            continue;
        }
        
        std::vector<Coords<float>> h_data;
#if BEFINE_CUSTOM_FORMAT
        h_data.resize(human_parts.size());
        std::size_t hp_idx = 0;
        for (const auto& hp: human_parts)
        {
            if (h.count(hp))
            {
                auto coord3d_ptr = h.at(hp);
                auto coord3d_vec = Coords<float>(
                    coord3d_ptr.begin(), coord3d_ptr.begin() + 3);
                auto coord3d_vec_rt = _platform.device()
                    .rototranslation(coord3d_vec);
                h_data[hp_idx++] = coord3d_vec_rt;
            }
            else 
            {
                h_data[hp_idx++] = {};
            }
        }
#else
        h_data.resize(h.size());
        std::size_t kp_idx = 0;
        for (const auto& k: h)
        {
            auto coord3d_ptr = k.second;
            auto coord3d_vec = Coords<float>(
                coord3d_ptr.begin(), coord3d_ptr.begin() + 3);
            auto coord3d_vec_rt = _platform.device()
                .rototranslation(coord3d_vec);
            h_data[kp_idx++] = coord3d_vec_rt;
        }
#endif

        // Calculate mid shoulders.
        if (_mid_shoulder_idx != -1 
            && h.count(HP_SHOULDERS.at(0)) && h.count(HP_SHOULDERS.at(1)))
        {
            h_data[_mid_shoulder_idx] = Math::arr_mean(
                h_data.at(_shoulders_idx.at(0)), h_data.at(_shoulders_idx.at(1))
            );
        }

        // Calculate mid hips.
        if (_mid_hip_idx != -1 
            && h.count(HP_HIPS.at(0)) && h.count(HP_HIPS.at(1)))
        {
            h_data[_mid_hip_idx] = Math::arr_mean(
                h_data.at(_hips_idx.at(0)), h_data.at(_hips_idx.at(1))
            );
        }

        data.push_back(h_data);
    }
    return data;
}

std::tuple<cv::Mat, cv::Mat, std::uint64_t> 
MqttMaeve::_skip_frames_until(std::int64_t time_ns)
{
    if (time_ns == -1)
    {
        return {cv::Mat(), cv::Mat(), 0};
    }

    std::uint64_t time;
    cv::Mat color_frame;
    cv::Mat depth_frame;
    std::cout << "[" << Utils::timestamp_conv() << "]" 
        << " Start to skip until " 
        << Utils::timestamp_conv(time_ns * 1e-9) 
        << " (" << time_ns << " ns)"
        << std::endl;
    do {
        _cam_times.start();
        auto frameset = _platform.get_frame();
        _cam_times.stop();
        color_frame = std::get<0>(frameset);
        depth_frame = std::get<1>(frameset);
        time = std::get<2>(frameset);
        if (time == 0) 
        {
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(1ms);
        }
        std::cout << "[" << Utils::timestamp_conv() << "]"  
            << " Current frame timestamp " 
            << Utils::timestamp_conv(time * 1e-9) 
            << " (" << time << " ns)"
            << std::endl;
    } while(time < time_ns && !g_exit_flag);

    return {color_frame, depth_frame, time};
}

void MqttMaeve::_wait_start_until(std::time_t time)
{
    auto time_str = Utils::timestamp_conv(time);
    if (std::time(0) >= time)
    {
        std::cout << "[WARNING] Wait start until " 
            << time_str 
            << " has already passed the present time " 
            << Utils::timestamp_conv() << std::endl;
    }
    else
    {
        std::cout << "[" << Utils::timestamp_conv() << "]" 
            << " Start to wait until " << time_str 
            << " (" << std::uint64_t(time * 1e9) << " ns)"
            << std::endl;
        // std::this_thread::sleep_until(
        //     std::chrono::system_clock::from_time_t(time));
        while (std::time(0) < time) { }
        std::cout << "[" << Utils::timestamp_conv() << "]" 
            << " Time reached "
            << "(" << std::uint64_t(
                std::chrono::high_resolution_clock::now()
                    .time_since_epoch().count()) << " ns)"
            << std::endl;
    }
}

bool MqttMaeve::_exec(
    json& obj,
    std::tuple<cv::Mat, cv::Mat, std::uint64_t> frameset,
    std::size_t frame_id,
    std::size_t iter,
    bool exit_on_stream_failure)
{
    _exec_times.start();
    auto color_frame = std::get<0>(frameset);
    auto depth_frame = std::get<1>(frameset);
    auto timestamp   = std::get<2>(frameset); 
    auto shape = color_frame.size();

    if (_platform.device().is_file_enabled() 
        && (color_frame.empty() || depth_frame.empty()))
    {
        if (exit_on_stream_failure)
        {
            std::cout << "[STATUS] Stream finished with empty "
                "color frame or empty depth frame: exit program loop." 
                << std::endl;
            return true;
        }
        else 
        {
            std::cout << "[WARNING] Stream returned an empty "
                "color frame or empty depth frame: wait." 
                << std::endl;
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(1ms);
            return false;
        }
    }
    else if (color_frame.empty() || depth_frame.empty())
    {
        std::cout << "[WARNING] Empty color frame or empty depth frame." 
            << std::endl;
        return false;
    }
    
    auto kp2d = _platform.get_2D_keypoints(color_frame);
    auto kp3d = _platform.get_3D_keypoints(depth_frame, kp2d);

#if 0
    cv::imwrite("color_bgra_" + std::to_string(frame_id) + ".tiff", 
        color_frame);
    cv::imwrite("depth_fp32_" + std::to_string(frame_id) + ".tiff", 
        depth_frame);
    if (kp3d.size() > 0)
    {
        for (const auto& e: kp3d[0])
        {
            cv::Mat depth_mat;
            int x = e.second.at(4);
            int y = e.second.at(3);
            depth_mat = cv::imread(
                "depth_fp32_" + std::to_string(frame_id) + ".tiff", 
                cv::IMREAD_UNCHANGED);
            std::cout << e.first << " - " << frame_id << " - "
                << "x: " << x << ", y: " << y
                << ", d_produced: " << e.second.at(5) 
                << ", d_ref: " << depth_mat.at<float>(y, x)
                << ", d_produced_1: " << depth_frame.at<float>(y, x)
                << std::endl;
        }
    }
#endif

    auto data = _postprocessing(kp3d, _human_parts);

    auto timestamp_pub = std::uint64_t(
        std::chrono::high_resolution_clock::now()
            .time_since_epoch().count());
    obj["continuousState"] = data;
#if DEBUG
    obj["kp2d"] = kp2d;
    obj["kp3d"] = kp3d;
#endif
    obj["frame_id"] = frame_id;
    obj["timestamp"] = double(timestamp) * 1e-6;
    obj["timestamp_pub"] = double(timestamp_pub) * 1e-6;
        
    _exec_times.stop();

    std::cout 
        << "[" << Utils::timestamp_conv() <<  "] { " 
        << "iter: " << iter << "; "
        << "frame_id: " << frame_id << "; "
        << "t_frame: " << timestamp << " ns; "
        << "t_pub: " << timestamp_pub << " ns; " 
        << "frame shape: " << shape << " }"
        << std::endl;

    return false;
}

void MqttMaeve::_print_loop_stats(std::size_t frame_id, std::size_t iter)
{
#if DEBUG
    std::cout 
        << "[" << Utils::timestamp_conv() <<  "] { " 
        << "iter: " << iter << "; "
        << "frame_id: " << frame_id << "; "
        << "loop_time: " << _loop_times.elapsed() << " ms; "
        << "cam_time: " << _cam_times.elapsed() << " ms; "
        << "exec_time: " << _exec_times.elapsed() << " ms; "
        << "pub_time: " << _pub_times.elapsed() << " ms; "
        << "loop_fps: " 
            <<  (1.0 / _loop_times.elapsed()) * 1000.0 << " hz; "
        << "cam_fps: " 
            <<  (1.0 / _cam_times.elapsed()) * 1000.0 << " hz; "
        << "exec_fps: " 
            <<  (1.0 / _exec_times.elapsed()) * 1000.0 << " hz "
        << " }"<< std::endl;
#endif
}

} // namespace maeve
