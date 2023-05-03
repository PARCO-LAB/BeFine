/**
 * @file folder.cpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Folder device source code.
 */

#include "maeve/camera/folder/folder.hpp"

#include <opencv2/opencv.hpp>


namespace maeve {

Folder::Folder(std::vector<ImageSensor> sensors_list, std::string filepath,
    FolderSetting folder_setting)
    : Device(DeviceSource::FILE, sensors_list, filepath)
    , _device_setting{folder_setting}
    , _timestamp{0}
    , _frame_id{0}
    // , _fps{}
    , _index_csv{
        fs::path(filepath) / folder_setting.index_fn, 
        { parser::Type::AUTO }, ';'}
    , _index_csv_i{0}
    , _intrinsics_csv{fs::path(filepath) / folder_setting.intrinsics_fn, 
        { parser::Type::AUTO }, ';'}
{
    fs::path root(filepath);
    if (!fs::exists(root)) 
    {
        throw std::runtime_error("filepath do not exist");
    }
    if (!fs::is_directory(root))
    {
        throw std::runtime_error("filepath is not a directory");
    }

    fs::path color_fp = root / _device_setting.color_fn;
    if (!fs::exists(color_fp)) 
    {
        throw std::runtime_error("color_fn do not exist");
    }
    if (!fs::is_directory(color_fp))
    {
        throw std::runtime_error("color_fn is not a directory");
    }

    fs::path depth_fp = root / _device_setting.depth_fn;
    if (!fs::exists(depth_fp)) 
    {
        throw std::runtime_error("depth_fn do not exist");
    }
    if (!fs::is_directory(depth_fp))
    {
        throw std::runtime_error("depth_fn is not a directory");
    }
}

Folder::~Folder()
{
}

void Folder::start()
{
    Device::start();
}

void Folder::stop()
{
    Device::stop();
}

std::vector<cv::Mat> Folder::frameset()
{
    // _fps.start();
    if (!_is_active)
    {
        start();
    }

    // Read frame_id and timestamp.
    if (_index_csv_i >= _index_csv.rows_size())
    {
        return {cv::Mat(), cv::Mat()};
    }

    auto row = _index_csv[_index_csv_i++];
    auto image_idx = row[0].as<std::size_t>();
    _timestamp = row[1].as<uint64_t>();
    _frame_id  = row[2].as<std::size_t>();
    
    // Prepare filepath.
    fs::path root{_filepath};
    fs::path color_fp{root / _device_setting.color_fn};
    color_fp /= _device_setting.color_prefix 
             + std::to_string(image_idx) + "." + _device_setting.color_ext;
    fs::path depth_fp{root / _device_setting.depth_fn};
    depth_fp /= _device_setting.depth_prefix 
             + std::to_string(image_idx) + "." + _device_setting.depth_ext;

    cv::Mat color_mat;
    cv::Mat depth_mat;
    for (ImageSensor& is: _sensors_list)
    {
        switch (is.type())
        {
            case SensorType::COLOR:
            {
                color_mat = cv::imread(color_fp.string());
                cv::resize(color_mat, color_mat, 
                    cv::Size(is.width(), is.height()));
                break;
            }
            case SensorType::DEPTH:
            {
                depth_mat = cv::imread(depth_fp.string(), cv::IMREAD_ANYDEPTH);
                cv::resize(depth_mat, depth_mat, 
                    cv::Size(is.width(), is.height()));
                break;
            }
            case SensorType::NONE:
            default:
            {
                break;
            }
        }
    }
    // _fps.stop();
    return {color_mat, depth_mat};
}

uint64_t Folder::timestamp()
{
    return _timestamp;
}

float Folder::fps() 
{
    return _device_setting.fps;
}

} // namespace maeve
