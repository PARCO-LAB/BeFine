/**
 * @file realsense.cpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief RealSense device source code.
 */

#include "maeve/camera/realsense/realsense.hpp"
#include "maeve/camera/sensor.hpp"

#include <librealsense2/rsutil.h>

#include <exception>
#include <stdexcept>


namespace maeve {

RealSense::RealSense(DeviceSource source, 
    std::vector<ImageSensor> sensors_list, std::string filepath, 
    RealSenseSetting realsense_setting)
    : Device(source, sensors_list, filepath)
    , _device_setting{realsense_setting}
    , _frameset{}
    , _pipeline{}
    , _profile{}
{ }

RealSense::RealSense(std::vector<ImageSensor> sensors_list, 
    RealSenseSetting realsense_setting)
    : Device(sensors_list)
    , _device_setting{realsense_setting}
    , _frameset{}
    , _pipeline{}
    , _profile{}
{ }

RealSense::RealSense(std::string filepath, RealSenseSetting realsense_setting)
    : Device(filepath)
    , _device_setting{realsense_setting}
    , _frameset{}
    , _pipeline{}
    , _profile{}
{ }

RealSense::~RealSense()
{
    if (_is_active)
    {
        _pipeline.stop();
    }
}

void RealSense::align(RealSenseAlign align) 
{
    _device_setting.align = align;
}

rs2::align RealSense::align() 
{
    switch (_device_setting.align)
    {
        case RealSenseAlign::COLOR: 
            return rs2::align(_device_setting.stream_color);
        case RealSenseAlign::DEPTH: 
            return rs2::align(_device_setting.stream_depth);
        case RealSenseAlign::NONE:  
            return rs2::align(_device_setting.stream_none);
        default: throw std::domain_error("'align' field not recognized");
    }
}

void RealSense::start()
{
    rs2::config cfg = _setup();
    _profile = _pipeline.start(cfg);
    Device::start();

    _setup_params();
}

void RealSense::stop()
{
    _pipeline.stop();
    _profile = rs2::pipeline_profile();
    
    _params_loaded    = false;
    _intrinsic_loaded = false;
    _extrinsic_loaded = false;
    
    Device::stop();
}

std::vector<cv::Mat> RealSense::frameset()
{
    if (!_is_active)
    {
        start();
    }

    _frameset = _pipeline.wait_for_frames(_device_setting.frame_timeout);
    _frameset = _device_setting.align != RealSenseAlign::NONE ? 
        align().process(_frameset) : _frameset;

    cv::Mat color_mat;
    if (_is_rgb_enabled || _is_file_enabled)
    {
        rs2::video_frame color_frame = _frameset.get_color_frame()
            .as<rs2::video_frame>();
        const size_t color_width = color_frame.get_width();
        const size_t color_height = color_frame.get_height();
        color_mat = cv::Mat(cv::Size(color_width, color_height), 
            _device_setting.format_color_cv, (void *) color_frame.get_data(), 
            color_frame.get_stride_in_bytes());
    }
    
    cv::Mat depth_mat;
    if (_is_depth_enabled || _is_file_enabled)
    {
        rs2::video_frame depth_frame = _frameset.get_depth_frame()
            .as<rs2::video_frame>();
        const size_t depth_width = depth_frame.get_width();
        const size_t depth_height = depth_frame.get_height();
        depth_mat = cv::Mat(cv::Size(depth_width, depth_height), 
            _device_setting.format_depth_cv, (void *) depth_frame.get_data(), 
            depth_frame.get_stride_in_bytes());
    }
    
    return {color_mat, depth_mat};
}

uint64_t RealSense::timestamp()
{
    return uint64_t(_frameset.get_timestamp() * 1000000ULL);
}

float* RealSense::deproject(float* coord_3d, const float* coord, float d) const
{
    auto intrinsics = rs_intrinsics();
    rs2_deproject_pixel_to_point(coord_3d, &intrinsics, coord, d);
    return coord_3d;
}

float RealSense::fps() const
{
    if (!_is_active)
    {
        throw std::runtime_error("the device is not active, "\
                                 "ensure to start the pipeline");
    }
    return static_cast<float>(_profile.get_stream().fps());
}

rs2::pipeline_profile RealSense::profile() const
{
    return _profile;
}

rs2_intrinsics RealSense::rs_intrinsics() const 
{
    // // Check if profile is valid.
    // if (_profile)
    // {
    //     throw std::runtime_error(
    //         "the device profile is not valid, ensure to start the pipeline");
    // }

    if (!_is_active)
    {
        throw std::runtime_error("the device is not active, "\
                                 "ensure to start the pipeline");
    }

    // Check if profile contains a valid video stream profile. 
    if (auto stream_profile = _profile.get_stream(_device_setting.stream_color)
            .as<rs2::video_stream_profile>())
    {
        return stream_profile.get_intrinsics();
    }
    else 
    {
        throw std::runtime_error("the device profile doesn't contain a video "\
            "stream profile, ensure to use a device with this feature");
    }
}

float RealSense::get_depth_scale() const
{
    if (!_is_active)
    {
        throw std::runtime_error("the device is not active, "\
                                 "ensure to start the pipeline");
    }

    // Check if profile contains a valid video stream profile. 
    if (auto depth_sensor = _profile.get_device().first<rs2::depth_sensor>())
    {
        return depth_sensor.get_depth_scale();
    }
    else 
    {
        throw std::runtime_error("the device profile doesn't contain a depth "\
            "sensor, ensure to use a device with this feature");
    }
}

rs2::config RealSense::_setup()
{
    rs2::config ret;
    ret.disable_all_streams();

    switch (_source)
    {
        case DeviceSource::FILE:
        {
            if (_filepath.empty())
            {
                throw std::invalid_argument("'filepath' argument has "\
                                            "not to be empty");
            }
            ret.enable_device_from_file(_filepath);
            break;
        }
        case DeviceSource::CAMERA:
        {
            for (ImageSensor& s: _sensors_list)
            {
                switch (is.type())
                {
                    case SensorType::COLOR:
                    {
                        ret.enable_stream(
                            _device_setting.stream_color, 
                            is.width(), is.height(), 
                            _device_setting.format_color, 
                            is.framerate());
                        break;
                    }
                    case SensorType::DEPTH:
                    {
                        ret.enable_stream(
                            _device_setting.stream_depth, 
                            is.width(), is.height(), 
                            _device_setting.format_depth, 
                            is.framerate());
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
            throw std::domain_error("'source' field not recognized");
        }
    }

    return ret;
}

void RealSense::_setup_params()
{
    if (!_params_loaded)
    {
        auto i = rs_intrinsics();
        _params.width  = i.width;
        _params.height = i.height;
        _params.fx     = i.fx;
        _params.fy     = i.fy;
        _params.ppx    = i.ppx;
        _params.ppy    = i.ppy;
        _params_loaded = true;
    }

    if (!_intrinsic_loaded)
    {
        auto i = rs_intrinsics();
        _params.K[0] = i.fx; _params.K[1] = 0.0;  _params.K[2] = i.ppx;
        _params.K[3] = 0.0;  _params.K[4] = i.fy; _params.K[5] = i.ppy;
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
