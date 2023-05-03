/**
 * @file zed.cpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Zed device source code.
 */

#include "maeve/camera/zed/zed.hpp"
#include "maeve/camera/zed/zed_utils.hpp"
#include "maeve/camera/sensor.hpp"

#include <exception>
#include <stdexcept>
#include <string>
#include <thread>
#include <chrono>


namespace maeve {

Zed::Zed(DeviceSource source, 
    std::vector<ImageSensor> sensors_list, std::string filepath,
    ZedSetting zed_setting)
    : Device(source, sensors_list, filepath)
    , _zed{}
    , _frameset{}
    , _device_setting{zed_setting}
{ }

Zed::Zed(std::vector<ImageSensor> sensors_list, ZedSetting zed_setting)
    : Device(sensors_list)
    , _zed{}
    , _frameset{}
    , _device_setting{zed_setting}
{ }

Zed::Zed(std::string filepath, ZedSetting zed_setting)
    : Device(filepath)
    , _zed{}
    , _frameset{}
    , _device_setting{zed_setting}
{ }

Zed::~Zed()
{
    if (_is_active)
    {
        _zed.close();
    }
}

void Zed::start()
{
    auto conf = _setup();
    auto returned_state = _zed.open(conf.init_params);
    std::size_t i = 0;
    while (returned_state != sl::ERROR_CODE::SUCCESS) 
    {
        // using namespace std::chrono_literals;
        // std::this_thread::sleep_for(1ms);
        std::cout << "[WARNING] Camera not open (" 
            << returned_state <<  "): retrying... " << std::endl;
        returned_state = _zed.open(conf.init_params);
        if ((_device_setting.timeout_s * 1000) <= i++)
        {
            throw std::runtime_error("cannot open ZED device");
        }
    }
    returned_state = _zed.enablePositionalTracking(conf.tracking_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) 
    {
        throw std::runtime_error(
            "cannot enablePositionalTracking of ZED device");    
    }
    Device::start();

    _setup_params();
}

void Zed::stop()
{
    _zed.close();
    Device::stop();
}

std::vector<cv::Mat> Zed::frameset()
{
    if (!_is_active)
    {
        start();
    }

    auto returned_state = _zed.grab(_frameset.runtime_params);
    if (returned_state != sl::ERROR_CODE::SUCCESS) 
    {
        return {cv::Mat(), cv::Mat()};
    }

    cv::Mat color_mat, depth_mat;
    if (_frameset.color_resolution.area() != 0
        && _frameset.depth_resolution.area() != 0)
    {
        sl::Mat color(
            _frameset.color_resolution.width, 
            _frameset.color_resolution.height, 
            _device_setting.color_mat_type);
        sl::Mat depth(
            _frameset.depth_resolution.width, 
            _frameset.depth_resolution.height, 
            _device_setting.depth_mat_type);

        color_mat = slMat2cvMat(color);
        depth_mat = slMat2cvMat(depth);

        if (_is_rgb_enabled || _is_file_enabled)
        {
            _zed.retrieveImage(color, sl::VIEW::LEFT, sl::MEM::CPU, 
                _frameset.color_resolution);
            cv::cvtColor(color_mat, color_mat, cv::COLOR_BGRA2BGR);
        }
        
        if (_is_depth_enabled || _is_file_enabled)
        {
            _zed.retrieveMeasure(depth, sl::MEASURE::DEPTH, sl::MEM::CPU,
                _frameset.depth_resolution);
        } 
    }
    else 
    {
        sl::Mat color;
        if (_is_rgb_enabled || _is_file_enabled)
        {
            _zed.retrieveImage(color, sl::VIEW::LEFT, sl::MEM::CPU);
            color_mat = slMat2cvMat(color);
            cv::cvtColor(color_mat, color_mat, cv::COLOR_BGRA2BGR);
        }
        
        sl::Mat depth;
        if (_is_depth_enabled || _is_file_enabled)
        {
            _zed.retrieveMeasure(depth, sl::MEASURE::DEPTH, sl::MEM::CPU);
            depth_mat = slMat2cvMat(depth);
        }   
    }
    
    return {color_mat.clone(), depth_mat.clone()};
}

uint64_t Zed::timestamp()
{
    return _zed.getTimestamp(sl::TIME_REFERENCE::IMAGE);
}

float Zed::fps() 
{
    if (!_is_active || !_zed.isOpened())
    {
        throw std::runtime_error("the device is not active, "\
                                 "ensure to open the zed");
    }
    return _zed.getCameraInformation().camera_configuration.fps;
}

sl::CameraParameters Zed::zed_camera_parameters()
{
    if (!_is_active || !_zed.isOpened())
    {
        throw std::runtime_error("the device is not active, "\
                                 "ensure to open the zed");
    }

    return _zed.getCameraInformation()
        .camera_configuration.calibration_parameters.left_cam;
}

ZedConfig Zed::_setup()
{
    ZedConfig c;

    c.init_params.camera_resolution = _device_setting.resolution; 
    c.init_params.coordinate_units = _device_setting.unit;
    c.init_params.camera_disable_self_calib = true;

    switch (_source)
    {
        case DeviceSource::FILE:
        {
            if (_filepath.empty())
            {
                throw std::invalid_argument("'filepath' argument has "\
                                            "not to be empty");
            }

            if (_device_setting.streaming)
            {   
                std::vector<std::string> uri = split(_filepath, ':');
                sl::String ip(uri.at(0).c_str());
                if (uri.size() == 2) 
                {
                    auto port = std::atoi(uri.at(1).c_str());
                    c.init_params.input.setFromStream(ip, port);
                }
                else 
                {   
                    auto port = 30000U;
                    c.init_params.input.setFromStream(ip, port);
                }
            }
            else 
            {
                c.init_params.input.setFromSVOFile(_filepath.c_str());
            }

            switch (_device_setting.resolution)
            {
                case sl::RESOLUTION::VGA:
                {
                    auto r = sl::Resolution(672, 376);
                    _frameset.depth_resolution = r;
                    _frameset.color_resolution = r;
                    break;
                }
                case sl::RESOLUTION::HD720:
                {
                    auto r = sl::Resolution(1280, 720);
                    _frameset.depth_resolution = r;
                    _frameset.color_resolution = r;
                    break;
                }
                case sl::RESOLUTION::HD1080:
                {
                    auto r = sl::Resolution(1920, 1080);
                    _frameset.depth_resolution = r;
                    _frameset.color_resolution = r;
                    break;
                }
                case sl::RESOLUTION::HD2K:
                {
                    auto r = sl::Resolution(2208, 1242);
                    _frameset.depth_resolution = r;
                    _frameset.color_resolution = r;
                    break;
                }
                default:
                {
                    break;   
                }
            }

            c.init_params.depth_mode = _device_setting.depth_mode;
            c.init_params.depth_stabilization = true;
            _frameset.runtime_params.sensing_mode 
                = _device_setting.sensing_mode;
            c.tracking_params.set_as_static = true;
            break;
        }
        case DeviceSource::CAMERA:
        {
            for (ImageSensor& is: _sensors_list)
            {
                switch (is.type())
                {
                    case SensorType::COLOR:
                    {
                        c.init_params.camera_fps = is.framerate(); 
                        _frameset.color_resolution = sl::Resolution(
                            is.width(), is.height());
                        break;
                    }
                    case SensorType::DEPTH:
                    {
                        if (!_is_rgb_enabled) 
                        {
                            c.init_params.camera_fps = is.framerate(); 
                        }
                        c.init_params.depth_mode = _device_setting.depth_mode;
                        c.init_params.depth_stabilization = true;
                        _frameset.runtime_params.sensing_mode 
                            = _device_setting.sensing_mode;
                        c.tracking_params.set_as_static = true;
                        _frameset.depth_resolution = sl::Resolution(
                            is.width(), is.height());
                        break;
                    }
                    case SensorType::NONE:
                    default:
                    {
                        break;
                    }
                }
            }
            break;
        }
        default:
        {
            throw std::domain_error("'source' field not recogni_zed");
        }
    }

    return c;
}

void Zed::_setup_params()
{
    if (!_params_loaded)
    {
        auto i = zed_camera_parameters();
        _params.width  = i.image_size.width;
        _params.height = i.image_size.height;
        _params.fx     = i.fx;
        _params.fy     = i.fy;
        _params.ppx    = i.cx;
        _params.ppy    = i.cy;
        _params_loaded = true;
    }

    if (!_intrinsic_loaded)
    {
        auto i = zed_camera_parameters();
        _params.K[0] = i.fx; _params.K[1] = 0.0;  _params.K[2] = i.cx;
        _params.K[3] = 0.0;  _params.K[4] = i.fy; _params.K[5] = i.cy;
        _params.K[6] = 0.0;  _params.K[7] = 0.0;  _params.K[8] = 1.0;
        _intrinsic_loaded = true;
    }

    if (!_extrinsic_loaded)
    {
        _params.RT[0] = 1.0; _params.RT[1] = 0.0; _params.RT[2] = 0.0;
        _params.RT[3] = 0.0; _params.RT[4] = 1.0; _params.RT[5] = 0.0;
        _params.RT[6] = 0.0; _params.RT[7] = 0.0; _params.RT[8] = 1.0;
        _extrinsic_loaded = true;
    }
}

} // namespace maeve
