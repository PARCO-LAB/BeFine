/**
 * @file zed.hpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief ZED device header code.
 */

#ifndef CAMERA_ZED_ZED_HPP
#define CAMERA_ZED_ZED_HPP

#include "maeve/camera/sensor.hpp"
#include "maeve/camera/device.hpp"

#include <sl/Camera.hpp>

#include <vector>


namespace maeve {

class Zed;

struct ZedSetting : public DeviceSetting {
    sl::RESOLUTION   resolution     = sl::RESOLUTION::HD2K;
    sl::UNIT         unit           = sl::UNIT::MILLIMETER;
    sl::DEPTH_MODE   depth_mode     = sl::DEPTH_MODE::PERFORMANCE;
    sl::SENSING_MODE sensing_mode   = sl::SENSING_MODE::FILL;
    sl::MAT_TYPE     color_mat_type = sl::MAT_TYPE::U8_C4;
    sl::MAT_TYPE     depth_mat_type = sl::MAT_TYPE::F32_C1;
    bool             streaming      = false;
    std::size_t      timeout_s      = 10;
};

struct ZedConfig
{   
    sl::InitParameters init_params;
    sl::PositionalTrackingParameters tracking_params;
};

struct ZedFrameset
{
    sl::Resolution color_resolution;
    sl::Resolution depth_resolution;
    sl::RuntimeParameters runtime_params;
};

/**
 * @brief Class for Zed camera configuration.
 */
class Zed : public Device
{
public:
    /**
     * @brief Construct a new Zed object
     * @param source       Source stream type. See DeviceSource.
     * @param sensors_list List of ImageSensor objects.
     * @param filepath     Filepath of rosbag to use.
     * @param zed_setting  Additional camera setting.
     */
    Zed(DeviceSource source = DeviceSource::CAMERA, 
        std::vector<ImageSensor> sensors_list = {}, 
        std::string filepath = std::string(),
        ZedSetting zed_setting = ZedSetting{});
    
    /**
     * @brief Construct a new Zed object for camera usage.
     * @param sensors_list List of ImageSensor objects.
     * @param zed_setting  Additional camera setting.
     */
    Zed(std::vector<ImageSensor> sensors_list,
        ZedSetting zed_setting = ZedSetting{});

    /**
     * @brief Construct a new Real Sense object for file rosbag usage.
     * @param filepath    Filepath of rosbag to use.
     * @param zed_setting Additional camera setting.
     */
    Zed(std::string filepath, ZedSetting zed_setting = ZedSetting{});

    /**
     * @brief Destroy the Zed object.
     * Stop the realsense device.
     */
    ~Zed() override;

    /**
     * @brief Start the Zed device using the setted configuration.
     */
    void start() override;

    /**
     * @brief Stop the Zed device.
     */
    void stop() override;

    /**
     * @brief Start the device, wait for new image frames and return it.
     * @return std::vector<cv::Mat> The first element is the RGB cv::Mat, the 
     * second element is the Depth cv::Mat.
     */
    std::vector<cv::Mat> frameset() override;

    /**
     * @brief Get the timestamp of the last frame retrived with the 
     * frameset function call.
     * @return uint64_t nanoseconds. 
     */
    uint64_t timestamp() override;

    /**
     * @brief Get the Realsense camera FPS. 
     * @return float The FPS in floating point. 
     */
    float fps() override;

    std::size_t frame_id() override { return _zed.getSVOPosition(); } 

    /**
     * @brief Get Zed camera parameters.
     * @return sl::CameraParameters 
     */
    sl::CameraParameters zed_camera_parameters();

private:

    /**
     * @brief Setup the ZED device. 
     */
    ZedConfig _setup();

    /**
     * @brief Setup the device params according to RealSense API.
     */
    void _setup_params();

    sl::Camera _zed; ///< ZED Camera object.
    ZedFrameset _frameset; ///< ZED frameset.

    ZedSetting _device_setting; ///< Additional device setting. 
};

} // namespace maeve

#endif // CAMERA_ZED_ZED_HPP
