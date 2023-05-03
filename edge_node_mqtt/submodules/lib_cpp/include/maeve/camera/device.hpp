/**
 * @file device.hpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Device header code.
 */

#ifndef CAMERA_DEVICE_HPP
#define CAMERA_DEVICE_HPP

#include "maeve/camera/sensor.hpp"
#include "maeve/interpolation/common.hpp"
#include "maeve/path.hpp"

#include <opencv2/opencv.hpp>

#include <vector>


namespace maeve {

namespace fs = std::filesystem;


class Device;

/**
 * Enumeration to define source data retrieved by a RealSense device.
 */
enum class DeviceSource
{
    FILE,   ///< Images retrieved from a file.
    CAMERA  ///< Images retrieved from the camera.
};

struct DeviceSetting {};

/**
 * @brief Class for Device sensor configuration.
 */
class Device 
{
public:
    /**
     * @brief Construct a new Device object.
     */
    Device(DeviceSource source = DeviceSource::CAMERA, 
        std::vector<ImageSensor> sensors_list = {}, 
        std::string filepath = std::string(),
        DeviceSetting device_settings = DeviceSetting{});

    Device(std::vector<ImageSensor> sensors_list = {},
           DeviceSetting device_settings = DeviceSetting{});

    Device(std::string filepath = std::string(),
           DeviceSetting device_settings = DeviceSetting{});

    /**
     * @brief Destroy the Device object.
     */
    virtual ~Device() 
    { 
        if (_is_active)
        {
            stop(); 
        }
    }

    /**
     * @brief Class member "source" getter.
     * @return DeviceSource "source" member enum value.
     */
    const DeviceSource& source() const { return _source; }

    /**
     * @brief Class member "sensors_list" getter.
     * @return std::vector<ImageSensor> List of ImageSensor objects
     */
    virtual const std::vector<ImageSensor>& sensors_list() const 
    { 
        return _sensors_list; 
    }
    
    /**
     * @brief The device is active when the start method is called.
     * @return true if device is active, false if device is not.
     */
    virtual bool is_active() const { return _is_active; }

    /**
     * @brief Check if the device cannot be started or it can. 
     * @return true  The device cannot be started or it takes the default 
     *               configuration. 
     * @return false The device can be correctly started.
     */
    virtual bool empty() const;

    /**
     * @brief Camera calibration getter. 
     * @return CameraCalibration 
     */
    CameraCalibration<float> params() const { return _params; }

    /**
     * @brief Reconfigure the RealSense device. 
     * Warning: if device was active, this call will deactivate it.
     * @param source       Source stream type. See DeviceSource.
     * @param sensors_list List of ImageSensor objects.
     * @param filepath     Filepath of rosbag to use.
     */
    void config(DeviceSource source, 
        std::vector<ImageSensor> sensors_list = std::vector<ImageSensor>(), 
        std::string filepath = std::string());

    /**
     * @brief Set the intrinsic _params.K from file.
     * @param params_fp 
     * @param separator 
     */
    void set_intrinsic_params(fs::path params_fp, char separator = ',');

    /**
     * @brief Set the intrinsic _params.RT from file.
     * @param params_fp 
     * @param separator 
     */
    void set_extrinsic_params(fs::path params_fp, char separator = ',');

    /**
     * @brief Perform the rototranslation of the world point in relation to the
     * extrinsics parameters of the camera.
     * @param coord_rt  Destination pointer of rototraslated 3D coordinates
     * @param coord     Source 3D coordinates.
     * @return T* The destination pointer.
     */
    virtual float* rototranslation(
        float* coord_rt, const float* coord) const;

    /**
     * @brief Vector wrapper of rototranslation
     * @param coord 
     * @return std::vector<float> Empty vector on error.
     */
    virtual std::vector<float> rototranslation(
        const std::vector<float> coord) const;

    /**
     * @brief Deprojection from 2D to 3D world reference.
     * @param coord_3d Destination pointer of 3D coordinates.
     * @param coord    Source pointer of 2D coordinates.
     * @param d        Depth value in coord.
     * @return float* The destination pointer.
     */
    virtual float* deproject(
        float* coord_3d, const float* coord, float d) const;

    /**
     * @brief Vector wrapper of deproject.
     * @param coord 
     * @param d 
     * @return std::vector<float>  Empty vector on error.
     */
    virtual std::vector<float> deproject(
        const std::vector<float> coord, float d) const;

    /**
     * @brief Start the Device using the setted configuration.
     */
    virtual void start() { _is_active = true; };

    /**
     * @brief Stop the Device.
     */
    virtual void stop() { _is_active = false; };

    /**
     * @brief Get the list of frames retrieved from the device. 
     * @return std::vector<cv::Mat> The list of frames retrieved.
     */
    virtual std::vector<cv::Mat> frameset() = 0;

    /**
     * @brief Get the timestamp of the last frame retrieved.
     * @return uint64_t The timestamp in nanoseconds.
     */
    virtual uint64_t timestamp() { return 0ULL; }

    /**
     * @brief Get the camera configuration FPS. 
     * @return float The FPS in floating point. 
     */
    virtual float fps() { return 0.0; }

    virtual std::size_t frame_id() { return 0ULL; }

    /**
     * @brief Check if depth stream is enabled in Device device.
     * @return true if depth stream is enabled in Device device, otherwise 
     * false.
     */
    bool is_depth_enabled() const;

    /**
     * @brief Check if color stream is enabled in Device device.
     * @return true if color stream is enabled in Device device, otherwise 
     * false.
     */
    bool is_rgb_enabled() const;

    /**
     * @brief Check if file stream is enabled in Device device.
     * @return true if file stream is enabled in Device device, otherwise 
     * false.
     */
    bool is_file_enabled() const;

protected:
    /**
     * @brief String identifier of source stream type of Device device. 
     * Available values are: DeviceSource::CAMERA or 
     * DeviceSource::FILE. Default: DeviceSource::CAMERA.
     */
    DeviceSource        _source;

    /**
     * @brief Additional device settings.
     */
    DeviceSetting _device_setting;

    /**
     * @brief Camera calibration parameters.
     */
    CameraCalibration<float>   _params;
    
    /**
     * @brief True if _params field have been filled with the calibration 
     * parameters, otherwise false.
     */
    bool                _params_loaded;

    /**
     * @brief True if _params.K field have been filled, otherwise false.
     */
    bool                _intrinsic_loaded;

    /**
     * @brief True if _params.RT field have been filled, otherwise false.
     */
    bool                _extrinsic_loaded;

    /**
     * @brief List of ImageSensor objects to use for Device stream enabling. 
     * Used only if config is DeviceSource::CAMERA. 
     */
    std::vector<ImageSensor> _sensors_list; 

    /**
     * @brief Filepath of rosbag to use to configure the device. 
     * Used only if config is DeviceSource::FILE. 
     */
    std::string         _filepath;

    bool                _is_active; ///< When device is active, it is true.

    bool _is_depth_enabled; ///< If depth camera stream is enables, it is true.
    bool _is_rgb_enabled;   ///< If color camera stream is enables, it is true.
    bool _is_file_enabled;  ///< If file stream is enables, it is true.
};

} // namespace maeve

#endif // CAMERA_DEVICE_HPP
