/**
 * @file sensor.hpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Sensor header code.
 */

#ifndef CAMERA_SENSOR_HPP
#define CAMERA_SENSOR_HPP

#include <iostream>


namespace maeve {

/**
 * @brief Enumeration for sensor types.
 */
enum class SensorType
{
    DEPTH,  ///< Depth sensor. 
    COLOR,  ///< Color RGB sensor.
    NONE,   ///< Unknown sensor.
};

/**
 * @brief  Class interface for a generic Sensor configuration.
 */
class Sensor 
{
public: 
    /**
     * @brief Construct a new Sensor object.
     * @param type Sensor type.
     */
    Sensor(SensorType type = SensorType::NONE);

    /**
     * @brief Sensor copy constructor.
     * @param c Sensor object reference to copy.
     */
    Sensor(const Sensor &c);

    /**
     * @brief 'type' member getter.
     * @return rs2_stream value of 'type'.
     */
    virtual SensorType type() const { return _type; };

    /**
     * @brief 'type' member setter.
     * @param type New value of 'type'.
     */
    virtual void type(SensorType type) { this->_type = std::move(type); };

protected:
    SensorType _type;        ///< Image stream Sensor type.
};

class ImageSensor : public Sensor
{
public: 
    /**
     * @brief Construct a new ImageSensor object.
     * @param type Image sensor type.
     * @param height    Image height.
     * @param width     Image width.
     * @param framerate Image framerate.
     */
    ImageSensor(SensorType type, 
        uint32_t height, uint32_t width, uint32_t framerate);

    /**
     * @brief ImageSensor copy constructor
     * @param c ImageSensor object reference to copy.
     */
    ImageSensor(const ImageSensor &c);

    /**
     * @brief 'height' member getter.
     * @return uint32_t value of 'height'.
     */
    virtual uint32_t height() const { return _height; };

    /**
     * @brief 'height' member setter.
     * @param height New value of 'height'.
     */
    virtual void height(uint32_t height) { this->_height = std::move(height); };

    /**
     * @brief 'width' member getter.
     * @return uint32_t value of 'width'.
     */
    virtual uint32_t width() const { return _width; };

    /**
     * @brief 'width' member setter.
     * @param width New value of 'width'.
     */
    virtual void width(uint32_t width) { this->_width = std::move(width); };

    /**
     * @brief 'framerate' member getter.
     * @return uint32_t value of 'framerate'.
     */
    virtual uint32_t framerate() const { return _framerate; };

    /**
     * @brief 'framerate' member setter.
     * @param framerate New value of 'framerate'.
     */
    virtual void framerate(uint32_t framerate) 
    { 
        this->_framerate = std::move(framerate);
    };

private:
    uint32_t _height;        ///< Sensor height.
    uint32_t _width;         ///< Sensor width.
    uint32_t _framerate;     ///< Sensor framerate.
};

/**
 * @brief Class implementation of the color image sensor.
 */
class RgbSensor : public ImageSensor 
{
public:
    /**
     * @brief Construct a new RgbSensor object.
     * @param height    Image height.
     * @param width     Image width.
     * @param framerate Image framerate.
     */
    RgbSensor(uint32_t height, uint32_t width, uint32_t framerate);

    /**
     * @brief RgbSensor copy constructor
     * @param c RgbSensor object reference to copy.
     */
    RgbSensor(const RgbSensor &c);
};

/**
 * @brief Class implementation of the depth image sensor.
 */
class DepthSensor : public ImageSensor 
{
public:
    /**
     * @brief Construct a new DepthSensor object.
     * @param height    Image height.
     * @param width     Image width.
     * @param framerate Image framerate.
     */
    DepthSensor(uint32_t height, uint32_t width, uint32_t framerate);
    /**
     * @brief DepthSensor copy constructor
     * @param c DepthSensor object reference to copy.
     */
    DepthSensor(const DepthSensor &c);
};

} // namespace maeve

#endif // CAMERA_SENSOR_HPP
