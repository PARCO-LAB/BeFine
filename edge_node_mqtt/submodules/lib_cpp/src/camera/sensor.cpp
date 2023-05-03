/**
 * @file Sensor.cpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Sensor source code.
 */

#include "maeve/camera/sensor.hpp"


namespace maeve {

Sensor::Sensor(SensorType type)
    : _type{type}
{ }

Sensor::Sensor(const Sensor &c)
    : _type{c._type}
{ }

ImageSensor::ImageSensor(SensorType type, 
    uint32_t height, uint32_t width, uint32_t framerate)
    : Sensor(type)
    , _height{height}
    , _width{width}
    , _framerate{framerate}
{ }

ImageSensor::ImageSensor(const ImageSensor &c)
    : Sensor(c)
    , _height{c._height}
    , _width{c._width}
    , _framerate{c._framerate}
{ }

RgbSensor::RgbSensor(uint32_t height, uint32_t width, uint32_t framerate)
    : ImageSensor(SensorType::COLOR, height, width, framerate)
{ }

RgbSensor::RgbSensor(const RgbSensor &c)
    : ImageSensor(c)
{ }

DepthSensor::DepthSensor(uint32_t height, uint32_t width, uint32_t framerate)
    : ImageSensor(SensorType::DEPTH, height, width, framerate)
{ }

DepthSensor::DepthSensor(const DepthSensor &c)
    : ImageSensor(c)
{ }

} // namespace maeve
