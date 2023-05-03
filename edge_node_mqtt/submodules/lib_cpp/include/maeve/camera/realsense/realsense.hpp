/**
 * @file realsense.hpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief RealSense device header code.
 */

#ifndef CAMERA_REALSENSE_REALSENSE_HPP
#define CAMERA_REALSENSE_REALSENSE_HPP

#include "maeve/camera/sensor.hpp"
#include "maeve/camera/device.hpp"

#include <librealsense2/rs.hpp>

#include <vector>


namespace maeve {

class RealSense;

/**
 * Enumeration of RealSense image alignment configuration. 
 */
enum class RealSenseAlign 
{
    DEPTH,  ///< Image aligned to depth sensor. 
    COLOR,  ///< Image alighed to RGB sensor.
    NONE    ///< Do not perform align.
};

struct RealSenseSetting : public DeviceSetting {
    RealSenseAlign align              = RealSenseAlign::COLOR;
    uint32_t       frame_timeout      = 15000U;
    rs2_stream     stream_color       = RS2_STREAM_COLOR;
    rs2_stream     stream_depth       = RS2_STREAM_DEPTH;
    rs2_stream     stream_none        = RS2_STREAM_ANY;
    rs2_format     format_color       = RS2_FORMAT_RGB8;
    rs2_format     format_depth       = RS2_FORMAT_Z16;
    int            format_color_cv    = CV_8UC3;
    int            format_depth_cv    = CV_16UC1;
};

/**
 * @brief Class for RealSense camera configuration.
 */
class RealSense : public Device
{
public:
    /**
     * @brief Construct a new RealSense object
     * @param source            Source stream type. See DeviceSource.
     * @param sensors_list      List of ImageSensor objects.
     * @param filepath          Filepath of rosbag to use.
     * @param realsense_setting Additional camera setting.
     */
    RealSense(DeviceSource source = DeviceSource::CAMERA, 
        std::vector<ImageSensor> sensors_list = {}, 
        std::string filepath = std::string(),
        RealSenseSetting realsense_setting = RealSenseSetting{});
    
    /**
     * @brief Construct a new RealSense object for camera usage.
     * @param sensors_list       List of ImageSensor objects.
     * @param realsense_setting Additional camera setting.
     */
    RealSense(std::vector<ImageSensor> sensors_list, 
        RealSenseSetting realsense_setting = RealSenseSetting{});

    /**
     * @brief Construct a new Real Sense object for file rosbag usage.
     * @param filepath          Filepath of rosbag to use.
     * @param realsense_setting Additional camera setting.
     */
    RealSense(std::string filepath, 
        RealSenseSetting realsense_setting = RealSenseSetting{});

    /**
     * @brief Destroy the RealSense object.
     * Stop the realsense device.
     */
    ~RealSense() override;

    /**
     * @brief Class member 'align' setter.
     * @param align RealSenseAlign enum type.
     */
    void align(RealSenseAlign align);

    /**
     * @brief Class member 'align' getter.
     * @return rs2::align Align RealSense object with the selected stream.
     */
    rs2::align align();

    /**
     * @brief Start the RealSense device using the setted configuration.
     */
    void start() override;

    /**
     * @brief Stop the RealSense device.
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
     * @return uint64_t nanoseconds
     */
    uint64_t timestamp() override;

    /**
     * @brief Deprojection from 2D to 3D world reference with Realsense API. 
     * @param coord_3d Destination pointer of 3D coordinates.
     * @param coord    Source pointer of 2D coordinates.
     * @param d        Depth value in coord.
     * @return double* The destination pointer.
     */
    float* deproject(
        float* coord_3d, const float* coord, float d) const override;

    /**
     * @brief Get the Realsense camera FPS. 
     * @return float The FPS in floating point. 
     */
    float fps() const override;

    std::size_t frame_id() override { return _frameset.get_frame_number(); }

    /**
     * @brief Get RealSense device profile from current configuration and 
     * active pipeline. Warning: the profile could not be active.
     * @return rs2::pipeline_profile object.
     */
    rs2::pipeline_profile profile() const;

    /**
     * @brief Get the intrinsics object of the RealSense device.
     * @return rs2_intrinsics of the device.
     */
    rs2_intrinsics rs_intrinsics() const;

    /**
     * @brief Get the depth scale from the pipeline profile depth sensor.
     * @return float The depth scale.
     */
    float get_depth_scale() const;
    
private:

    /**
     * @brief Setup the Realsense device and return the configuration. 
     * @return rs2::config object.
     */
    rs2::config _setup();

    /**
     * @brief Setup the device params according to RealSense API.
     */
    void _setup_params();

    RealSenseSetting      _device_setting; ///< Additional camera setting. 

    rs2::frameset         _frameset;  ///< RealSense frameset.
    rs2::pipeline         _pipeline;  ///< RealSense pipeline.
    rs2::pipeline_profile _profile;   ///< RealSense profile.
};

} // namespace maeve

#endif // CAMERA_REALSENSE_REALSENSE_HPP
