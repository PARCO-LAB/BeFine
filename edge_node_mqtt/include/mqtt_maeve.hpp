/**
 * @file mqtt_maeve.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "maeve/parser/config.hpp"
#include "maeve/core.hpp"
#include "maeve/path.hpp"
#include "maeve/utils.hpp"
#include "maeve/math.hpp"
#include "maeve/analysis.hpp"

#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>

#include <string>
#include <tuple>
#include <fstream>


#ifndef MQTT_MAEVE_HPP
#define MQTT_MAEVE_HPP

#define BEFINE_CUSTOM_FORMAT 1

namespace maeve {

using json = nlohmann::json;

namespace fs = std::filesystem;

extern std::atomic<bool> g_exit_flag;

/**
 * @brief MQTT communication configuration parameters.
 */
struct MqttConfig {
    MqttConfig(parser::ConfigSection conf_section)
    {
        auto conf_uri = conf_section["uri"];
        uri = conf_uri.empty() ? 
            "tcp://localhost:1883" : conf_uri.as<std::string>();

        auto conf_topic = conf_section["topic"];
        topic = conf_topic.empty() ? 
            std::string() : conf_topic.as<std::string>();

        auto conf_clientid = conf_section["clientid"];
        client_id = conf_clientid.empty() ? 
            "" : conf_clientid.as<std::string>();

        auto conf_username = conf_section["username"];
        username = conf_username.empty() ? 
            std::string() : conf_username.as<std::string>();

        auto conf_password = conf_section["password"];
        password = conf_password.empty() ? 
            std::string() : conf_password.as<std::string>();
        
        auto conf_qos = conf_section["qos"];
        qos = conf_qos.empty() ? 
            1 : conf_qos.as<int>();
    }

    void store(fs::path fp)
    {
        std::ofstream file;
        file.open(fp, std::ofstream::out | std::ofstream::app);
        file << "[MQTT-OUTPUT-CONF]\n";
        file << "uri=" << uri << "\n";
        file << "topic=" << topic << "\n"; 
        file << "clientid=" << client_id << "\n"; 
        file << "username=" << username << "\n"; 
        file << "password=" << password << "\n"; 
        file << "qos=" << qos << "\n"; 
    }

    void print()
    {
        std::cout << "uri: " 
            << (uri.empty() ? "!empty!" : uri) << std::endl;
        std::cout << "topic: " 
            << (topic.empty() ? "!empty!" : topic) << std::endl; 
        std::cout << "clientid: " 
            << (client_id.empty() ? "!empty!" : client_id) << std::endl; 
        std::cout << "username: " 
            << (username.empty() ? "!empty!" : username) << std::endl; 
        std::cout << "password: " 
            << (password.empty() ? "!empty!" : password) << std::endl; 
        std::cout << "qos: " << qos << std::endl; 
    }

    std::string uri         = "tcp://localhost:1883";
    std::string topic       = std::string();
    std::string client_id   = "";
    std::string username    = std::string();
    std::string password    = std::string();
    int         qos         = 1;
};

/**
 * @brief Calibration configuration parameters.
 */
struct CalibrationConfig {
    CalibrationConfig(parser::ConfigSection conf_section)
    {
        auto conf_extrinsics_fp = conf_section["extrinsics-fp"];
        extrinsics_fp = conf_extrinsics_fp.empty() ? 
            std::string() : conf_extrinsics_fp.as<std::string>();

        auto conf_intrinsics_fp = conf_section["intrinsics-fp"];
        intrinsics_fp = conf_intrinsics_fp.empty() ? 
            std::string() : conf_intrinsics_fp.as<std::string>();

        auto conf_separator = conf_section["separator"];
        separator = conf_separator.empty() ? ' ' : conf_separator.as<char>();
    }

    void store(fs::path fp)
    {
        std::ofstream file;
        file.open(fp, std::ofstream::out | std::ofstream::app);
        file << "\n[CALIBRATION-OUTPUT-CONF]\n";
        file << "extrinsics-fp=" << extrinsics_fp << "\n";
        file << "intrinsics-fp=" << intrinsics_fp << "\n";
        file << "separator=" << separator << "\n"; 
    }

    void print()
    {
        std::cout << "extrinsics-fp: " 
            << (extrinsics_fp.empty() ? "!empty!" : extrinsics_fp.string()) 
            << std::endl;
        std::cout << "intrinsics-fp: " 
            << (intrinsics_fp.empty() ? "!empty!" : intrinsics_fp.string()) 
            << std::endl;
        std::cout << "separator: " << separator << std::endl; 
    }

    fs::path extrinsics_fp;
    fs::path intrinsics_fp;
    char separator;
};

/**
 * @brief RTSP configuration parameters.
 */
struct RtspConfig {
    RtspConfig(parser::ConfigSection conf_section)
    {
        auto conf_location = conf_section["location"];
        location = conf_location.empty() ? 
            std::string() : conf_location.as<std::string>();
    }

    void store(fs::path fp)
    {
        std::ofstream file;
        file.open(fp, std::ofstream::out | std::ofstream::app);
        file << "\n[RTSP-OUTPUT-CONF]\n";
        file << "location=" << location << "\n";
    }

    void print()
    {
        std::cout << "rtsp-location: " 
            << (location.empty() ? "!empty!" : location) 
            << std::endl;
    }

    std::string location;
};

/**
 * @brief MQTT MAEVE class.
 */
class MqttMaeve {
public:
    template <typename T>
    using BeFineMultiHumanKeypoints = std::vector<std::vector<Coords<T>>>;

    /**
     * @brief Construct a new Mqtt Maeve object.
     * @param platform 
     * @param conf_fp 
     * @param out_conf_fp 
     * @param skip_frames_until_time Timestamp in nanoseconds to reach skipping
     * frames of the camera.
     * @param wait_start_until_time String of the time to start the process 
     * in the following format "DD/MM/YYYY hh:mm:ss".
     */
    MqttMaeve(Maeve& platform, 
        fs::path conf_fp = fs::path(), 
        fs::path out_conf_fp = fs::path(),
        std::int64_t skip_frames_until_time = -1,
        std::string wait_start_until_time = std::string());

    /**
     * @brief Destroy the Mqtt Maeve object.
     */
    ~MqttMaeve();

    /**
     * @brief Loop on frames and publish kp3d.
     * @param exit_on_stream_failure When file stream fails exit the main loop,
     * otherwise continue. 
     */
    void run_kp3d(bool exit_on_stream_failure = true);
    void run_kp3d_stream_only(bool exit_on_stream_failure = true);

    /**
     * @brief Loop on frames and dump kp3d.
     * @param exit_on_stream_failure When file stream fails exit the main loop,
     * otherwise continue. 
     */
    void run_kp3d_dump(bool exit_on_stream_failure = true);

    /**
     * @brief Loop on frames and publish a specific kp3d.
     */
    void run_kp3d_frame(std::uint64_t tframe);

    /**
     * @brief Loop on frames received through RTSP and publish a specific kp3d.
     */
    void run_kp3d_rtsp();

private:

    /**
     * @brief 
     * @return std::tuple<cv::Mat, cv::Mat, std::uint64_t>
     */
    std::tuple<cv::Mat, cv::Mat, std::uint64_t>  _init();

    /**
     * @brief 
     * @param kp3d 
     * @param human_parts
     * @return BeFineMultiHumanKeypoints 
     */
    BeFineMultiHumanKeypoints<float> _postprocessing(
        const MultiHumanKeypoints<float>& kp3d, 
        const std::vector<std::string>& human_parts);

    /**
     * @brief Skip frames until a timestamp (int64 - nanoseconds).
     * @param time_ns Timestamp in nanoseconds.
     * @return std::tuple<cv::Mat, cv::Mat, std::uint64_t>
     */
    std::tuple<cv::Mat, cv::Mat, std::uint64_t> 
    _skip_frames_until(std::int64_t time_ns = -1);

    /**
     * @brief Wait until time. 
     * @param time time_t.
     */
    void _wait_start_until(std::time_t time);

    /**
     * @brief Exec the 2D extraction, the 3D interpolation and MQTT publish.
     * @param obj
     * @param frameset
     * @param frame_id 
     * @param iter 
     * @param exit_on_stream_failure 
     * @return true  Exit the loop.
     * @return false Continue the loop.
     */
    bool _exec(
        json& obj,
        std::tuple<cv::Mat, cv::Mat, std::uint64_t> frameset,
        std::size_t frame_id,
        std::size_t iter,
        bool exit_on_stream_failure = true);

    /**
     * @brief Print loop statistics.
     * @param frame_id 
     * @param iter 
     */
    void _print_loop_stats(std::size_t frame_id, std::size_t iter);

    /// Maeve platform instance.
    Maeve& _platform;
    fs::path _conf_fp;

    /// MQTT communication fields.
    MqttConfig _mqtt_conf;
    CalibrationConfig _calib_conf;
    RtspConfig _rtsp_conf;
    mqtt::async_client _cli;

    /// Configuration parameters.
    fs::path _out_conf_fp;
    std::int64_t _skip_frames_until_time;
    std::string _wait_start_until_time;

    analysis::TimeVector<> _init_times;
    analysis::TimeVector<> _pub_times;
    analysis::TimeVector<> _loop_times;
    analysis::TimeVector<> _cam_times;
    analysis::TimeVector<> _exec_times;
    std::vector<double> _received_amount;
    analysis::Matrix<double> _matrix_analysis;

    /// Human parts to produce.
    std::vector<std::string>  _human_parts;
    std::vector<std::int64_t> _shoulders_idx;
    std::vector<std::int64_t> _hips_idx; 
    std::int64_t _mid_shoulder_idx; 
    std::int64_t _mid_hip_idx; 

    const std::string DFLT_ROOT_TOPIC = "/mqttmaeve";
    const std::string DFLT_KP3D_TOPIC = "/kp3d";

    const std::size_t KP_AMOUNT_TO_EXCLUDE = 4;
    const std::vector<std::string> HP_SHOULDERS = {
        "left_shoulder", "right_shoulder"};
    const std::vector<std::string> HP_HIPS = {
        "left_hip", "right_hip"};
    const std::string HP_MID_SHOULDERS = "mid_shoulders";
    const std::string HP_MID_HIPS = "mid_hips";
#if BEFINE_CUSTOM_FORMAT
    const std::vector<std::string> HP_TO_INSERT = {
        HP_MID_SHOULDERS, HP_MID_HIPS
    };
    const std::vector<std::string> HP_TO_REMOVE = {
        "left_eye", "right_eye"
    };
#else
    const std::vector<std::string> HP_TO_INSERT = {};
    const std::vector<std::string> HP_TO_REMOVE = {};
#endif
};

} // namespace maeve

#endif // MQTT_MAEVE_HPP