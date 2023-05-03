/**
 * @file folder.hpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Folder device header code.
 */

#ifndef CAMERA_FOLDER_FOLDER_HPP
#define CAMERA_FOLDER_FOLDER_HPP

#include "maeve/camera/sensor.hpp"
#include "maeve/camera/device.hpp"
#include "maeve/path.hpp"
#include "maeve/analysis.hpp"
#include "maeve/parser/csv.hpp"

#include <vector>

namespace maeve {

namespace fs = std::filesystem;

class Folder;

struct FolderSetting : public DeviceSetting {
    std::string color_fn        = "color";          //< Color folder name.
    std::string depth_fn        = "depth";          //< Depth folder name.
    std::string color_ext       = "png";            //< Color images extension.
    std::string depth_ext       = "tiff";           //< Depth images extension.
    std::string color_prefix    = "color_";         //< Color images prefix.
    std::string depth_prefix    = "depth_";         //< Depth images prefix.
    std::string index_fn        = "index.csv";      //< Index CSV filename.
    std::string intrinsics_fn   = "intrinsics.csv"; //< Intrinsics CSV filename.
    float       fps             = 15;               //< Camera FPS.
};

/**
 * @brief Class for Folder camera configuration.
 */
class Folder : public Device
{
public:
    /**
     * @brief Construct a new Folder object
     * @param source         Source stream type. See DeviceSource.
     * @param sensors_list   List of ImageSensor objects.
     * @param filepath       Filepath of rosbag to use.
     * @param folder_setting Folder settings.
     */
    Folder(std::vector<ImageSensor> sensors_list = {}, 
        std::string filepath = std::string(),
        FolderSetting folder_setting = FolderSetting{});

    /**
     * @brief Destroy the Folder object.
     * Stop the realsense device.
     */
    ~Folder() override;

    /**
     * @brief Start the Folder device using the setted configuration.
     */
    void start() override;

    /**
     * @brief Stop the Folder device.
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

    std::size_t frame_id() override { return _frame_id; } 

private:

    FolderSetting _device_setting; ///< Additional device setting. 

    uint64_t _timestamp;
    std::size_t _frame_id;
    // analysis::Time _fps;

    parser::CSV _index_csv;
    std::size_t _index_csv_i;
    parser::CSV _intrinsics_csv;
};

} // namespace maeve

#endif // CAMERA_FOLDER_FOLDER_HPP
