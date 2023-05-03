/**
 * @file interpolation_3d.hpp
 * @date 29/08/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Interpolation 3D of 2D keypoints from a depth frame header.
 */

#ifndef INTERPOLATION_3D_INTERPOLATION_HPP
#define INTERPOLATION_3D_INTERPOLATION_HPP

#include "maeve/interpolation/common.hpp"
#include "maeve/camera/device.hpp"
#include "maeve/nn/model.hpp"
#include "maeve/math.hpp"

#include <map>
#include <vector>

#include <opencv2/opencv.hpp>


namespace maeve 
{ 

/**
 * @brief Class for Interpolation 3D.
 */
class Interpolation3D 
{
public: 
    /**
     * @brief Wrapper of deproject with CameraCalibration.
     * @tparam T 
     * @param coord_3d 
     * @param cc 
     * @param coord 
     * @param d 
     * @return T* 
     */
    template <typename T>
    static T* deproject(T coord_3d[3], const CameraCalibration<T>& cc, 
        const T coord[2], T d)
    {   
        return Math::deproject<T>(coord_3d, cc.K, coord, d);
    }

    /**
     * @brief Wrapper of deproject with Device.
     * @param coord_3d 
     * @param device
     * @param coord 
     * @param d 
     * @return float* 
     */
    static float* deproject(float coord_3d[3], const Device& device, 
        const float coord[2], float d)
    {
        return device.deproject(coord_3d, coord, d);
    }

    /**
     * @brief Vector wrapper of deproject with CameraCalibration.
     * @tparam T 
     * @param k 
     * @param coord 
     * @param d 
     * @return std::vector<T> 
     */
    template <typename T>
    std::vector<T> deproject(const CameraCalibration<T>& cc, 
        const std::vector<T> coord, T d)
    {
        std::vector<T> ret(std::size_t{3});
        deproject<T>(ret.data(), cc.K, coord.data(), d);
        return ret;
    }

    /**
     * @brief Vector wrapper of deproject with CameraCalibration.
     * @param device
     * @param coord 
     * @param d 
     * @return std::vector<float> 
     */
    std::vector<float> deproject(const Device& device, 
        const std::vector<float> coord, float d)
    {
        std::vector<float> ret(std::size_t{3});
        device.deproject(ret.data(), coord.data(), d);
        return ret;
    }

    /**
     * @brief Interpolation 3D from a color shape and a depth frame of 
     * 2D keypoints.
     * @param color_mat_size Color frame shape.
     * @param depth_mat      Depth frame.
     * @param kp2D           Keypoints 2D.
     * @param cc             Camera info.
     * @param scale          Scale factor of depth frame.
     * @return MultiHumanKeypoints<float> 
     */
    static MultiHumanKeypoints<float> run(
        cv::Size color_mat_size,
        const cv::Mat &depth_mat,
        const MultiHumanKeypoints<float> &kp2D,
        const CameraCalibration<float>& cc,
        float scale = 1.0);

    static MultiHumanKeypoints<float> run(
        cv::Size color_mat_size,
        const cv::Mat &depth_mat,
        const MultiHumanKeypoints<float> &kp2D,
        const Device& device,
        float scale = 1.0);
    
    /**
     * @brief Interpolation 3D from a color frame and a depth frame of 
     * 2D keypoints.
     * @param color_mat      Color frame.
     * @param depth_mat      Depth frame.
     * @param kp2D           Keypoints 2D.
     * @param cc             Camera info.
     * @param scale          Scale factor of depth frame.
     * @return MultiHumanKeypoints<float> 
     */
    static MultiHumanKeypoints<float> run(
        const cv::Mat &color_mat,
        const cv::Mat &depth_mat,
        const MultiHumanKeypoints<float> &kp2D,
        const CameraCalibration<float>& cc,
        float scale = 1.0);
    
    static MultiHumanKeypoints<float> run(
        const cv::Mat &color_mat,
        const cv::Mat &depth_mat,
        const MultiHumanKeypoints<float> &kp2D,
        const Device& device,
        float scale = 1.0);
};

} // namespace maeve 

#endif // INTERPOLATION_3D_INTERPOLATION_HPP
