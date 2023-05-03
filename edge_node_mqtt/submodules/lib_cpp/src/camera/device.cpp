/**
 * @file realsense.cpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Device source code.
 */

#include "maeve/camera/device.hpp"
#include "maeve/camera/sensor.hpp"
#include "maeve/math.hpp"

#include <exception>
#include <stdexcept>
#include <fstream>


namespace maeve {

Device::Device(DeviceSource source, 
    std::vector<ImageSensor> sensors_list, std::string filepath,
    DeviceSetting device_setting)
    : _source{source}
    , _device_setting{device_setting}
    , _params{}
    , _params_loaded{false}
    , _intrinsic_loaded{false}
    , _extrinsic_loaded{false}
    , _sensors_list{sensors_list}
    , _filepath{filepath}
    , _is_active{false}
    , _is_depth_enabled{false}
    , _is_rgb_enabled{false}
    , _is_file_enabled{false}
{ 
    if (source == DeviceSource::CAMERA)
    {
        for (Sensor& s: _sensors_list)
        {
            if (s.type() == SensorType::COLOR)
            {
                _is_rgb_enabled = true;
            }
            else if (s.type() == SensorType::DEPTH)
            {
                _is_depth_enabled = true;
            }
        }
    }
    _is_file_enabled = _source == DeviceSource::FILE;
}

Device::Device(std::vector<ImageSensor> sensors_list, 
    DeviceSetting device_setting)
    : Device(DeviceSource::CAMERA, sensors_list, std::string(), device_setting)
{ }

Device::Device(std::string filepath, DeviceSetting device_setting)
    : Device(DeviceSource::FILE, {}, filepath, device_setting)
{ }

bool Device::empty() const
{
    return _sensors_list.empty() && !_is_file_enabled;
}

void Device::config(DeviceSource source, 
    std::vector<ImageSensor> sensors_list, 
    std::string filepath)
{
    if (_is_active)
    {
        stop();
    }

    _source = source;
    _sensors_list = sensors_list;
    _filepath = filepath;
}

void Device::set_intrinsic_params(fs::path params_fp, char separator)
{
    auto file = std::ifstream{params_fp};
    if(!file.is_open() || !file.good()) 
    {
        throw std::runtime_error("Could not open intrinsics file");
    }

    std::string line;
    const std::size_t ROW_AMOUNT = 3, COL_AMOUNT = 3;
    std::size_t row = 0, col = 0;
    while(std::getline(file, line))
    {
        if (row >= ROW_AMOUNT) break;

        std::stringstream ss_line{line};
        std::string field;
        
        col = 0;
        while (std::getline(ss_line, field, separator))
        {
            if (col >= COL_AMOUNT) break;

            std::stringstream ss_field{field};
            ss_field >> _params.K[row * COL_AMOUNT + col];
            ++col;
        }

        ++row;
    }

    _intrinsic_loaded = true;
}

void Device::set_extrinsic_params(fs::path params_fp, char separator)
{
    auto file = std::ifstream{params_fp};
    if(!file.is_open() || !file.good()) 
    {
        throw std::runtime_error("Could not open extrinsics file");
    }

    std::string line;
    const std::size_t ROW_AMOUNT = 4, COL_AMOUNT = 4;
    std::size_t row = 0, col = 0;
    while(std::getline(file, line))
    {
        if (row >= ROW_AMOUNT) break;

        std::stringstream ss_line{line};
        std::string field;
        
        col = 0;
        while (std::getline(ss_line, field, separator))
        {
            if (col >= COL_AMOUNT) break;

            std::stringstream ss_field{field};
            ss_field >> _params.RT[row * COL_AMOUNT + col];
            ++col;
        }

        ++row;
    }

    _extrinsic_loaded = true;
}

float* Device::rototranslation(float* coord_rt, const float* coord) const
{
    if (!_extrinsic_loaded) return nullptr;
    return Math::rototranslation<float>(coord_rt, _params.RT, coord);
}

std::vector<float> Device::rototranslation(const std::vector<float> coord) const
{
    std::vector<float> ret;
    if (coord.size() < 3 || !_extrinsic_loaded) return ret;
    ret.resize(std::size_t(3));
    rototranslation(ret.data(), coord.data());
    return ret;
}

float* Device::deproject(float* coord_3d, const float* coord, float d) const
{
    if (!_intrinsic_loaded) return nullptr;
    return Math::deproject<float>(
        coord_3d, _params.K, coord, d);
}

std::vector<float> Device::deproject(
    const std::vector<float> coord, float d) const
{
    std::vector<float> ret;
    if (coord.size() < 2 || !_intrinsic_loaded) return ret;
    ret.resize(std::size_t{3});
    deproject(ret.data(), coord.data(), d);
    return ret;
}

bool Device::is_depth_enabled() const
{
    return _is_depth_enabled;
}

bool Device::is_rgb_enabled() const
{
    return _is_rgb_enabled;
}

bool Device::is_file_enabled() const
{
    return _is_file_enabled;
}


} // namespace maeve
