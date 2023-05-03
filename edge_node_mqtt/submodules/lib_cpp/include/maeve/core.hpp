/**
 * @file core.hpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Maeve platform: top-level header file includes all user headers..
 */

#ifndef MAEVE_HPP
#define MAEVE_HPP

#include "maeve/filter/filter.hpp"
#include "maeve/nn/dnn.hpp"
#include "maeve/camera/camera.hpp"
#include "maeve/interpolation/interpolation_3d.hpp"

#include <opencv2/opencv.hpp>

#include <tuple>
#include <map>
#include <string>

namespace maeve 
{ 

class Maeve 
{
public:
    /**
     * @brief Construct a new Maeve object with the core functionalities.
     * @param device Camera device.
     * @param dnn    Model model used to infer keypoints.
     */
    Maeve(Device& device, Model& dnn);

    /**
     * @brief Device getter.
     * @return Device& 
     */
    Device& device() { return _device; }

    /**
     * @brief Model getter.
     * @return Model& 
     */
    Model& model() { return _dnn; }

    /**
     * @brief Initialize the Maeve platform.
     */
    void init();

    /**
     * @brief Camera calibration getter: intrinsics and extrinsics values.
     * @return CameraCalibration 
     */
    CameraCalibration<float> calibration();

    /**
     * @brief Wait a frame from the RealSense camera and, according to 
     * RealSense device configuration, return a color frame, a depth frame and 
     * the timestamp of the frame retrived.
     * @return std::tuple<cv::Mat, cv::Mat, uint64_t>
     */
    std::tuple<cv::Mat, cv::Mat, uint64_t> get_frame();

    /**
     * @brief Get the 2D keypoints object.
     * @param color_frame RGB frame from camera.
     * @return MultiHumanKeypoints<float>
     */
    MultiHumanKeypoints<float> get_2D_keypoints(
        cv::Mat& color_frame);

    /**
     * @brief Get the 3D keypoints object.
     * @param color_frame RGB frame from camera.
     * @param depth_frame Depth frame from camera.
     * @param kp2D Human pose keypoints 2D.
     * @return MultiHumanKeypoints<float>
     */
    MultiHumanKeypoints<float> get_3D_keypoints(
        const cv::Mat& color_frame,
        const cv::Mat& depth_frame,
        const MultiHumanKeypoints<float> &kp2D);

    MultiHumanKeypoints<float> get_3D_keypoints(
        const cv::Mat& depth_frame,
        const MultiHumanKeypoints<float> &kp2D);

private:
    Device&  _device;            ///< Device camera.
    Model&   _dnn;               ///< DNN model.
    bool     _to_init;           ///< Platform to init or not.
    cv::Size _color_mat_size;    ///< Retrieved color cv::Mat size.

    // TODO: remove
    float _depth_scale;        ///< Depth scale to translate pixel to meters.
};


} // namespace maeve 

#endif // MAEVE_HPP
