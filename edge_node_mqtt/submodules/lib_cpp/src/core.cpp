/**
 * @file core.cpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Maeve platform: core functionalities source code.
 */

#include "maeve/core.hpp"

#include <exception>
#include <stdexcept>


namespace maeve {

Maeve::Maeve(Device& device, Model& dnn)
    : _device{device}
    , _dnn{dnn}
    , _depth_scale{1.0}
    , _to_init{true}
    , _color_mat_size{}
{ 
    
}

CameraCalibration<float> Maeve::calibration()
{
    return _device.params();
}

void Maeve::init()
{
    if (!_to_init) return;
    if (!_device.empty()) _device.start();
    _dnn.load();
    _to_init = false;
}

std::tuple<cv::Mat, cv::Mat, uint64_t> Maeve::get_frame()
{
    if (_to_init) init(); 
    auto frame = _device.frameset();
    uint64_t timestamp = _device.timestamp();
    _color_mat_size = frame[0].size();
    return {frame[0], frame[1], timestamp};
}

MultiHumanKeypoints<float> Maeve::get_2D_keypoints(
    cv::Mat& color_frame)
{
    if (_to_init) init();
    return _dnn.exec_to_map(color_frame);
}  

MultiHumanKeypoints<float> Maeve::get_3D_keypoints(
    const cv::Mat& color_frame,
    const cv::Mat& depth_frame,
    const MultiHumanKeypoints<float> &kp2D)
{
    if (_to_init) init();
    return Interpolation3D::run(color_frame, depth_frame, kp2D, _device, 
        _depth_scale);
}

MultiHumanKeypoints<float> Maeve::get_3D_keypoints(
    const cv::Mat& depth_frame,
    const MultiHumanKeypoints<float> &kp2D)
{
    if (_to_init) init();
    return Interpolation3D::run(_color_mat_size, depth_frame, kp2D, _device, 
        _depth_scale);
}

} // namespace maeve
