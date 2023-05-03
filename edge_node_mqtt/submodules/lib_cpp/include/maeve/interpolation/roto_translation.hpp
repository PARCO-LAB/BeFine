/**
 * @file roto_translation.hpp
 * @date 7/12/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Roto translation of 3D coordinates in world reference.
 */

#ifndef INTERPOLATION_ROTO_TRANSLATION_HPP
#define INTERPOLATION_ROTO_TRANSLATION_HPP

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
 * @brief Class for RotoTranslation.
 */
class RotoTranslation 
{
public: 
    /**
     * @brief Wrapper of run with CameraCalibration.
     * @tparam T 
     * @param coord_rt 
     * @param cc 
     * @param coord 
     * @return T* 
     */
    template <typename T>
    static T* run(T coord_rt[3], const CameraCalibration<float>& cc, 
        const T coord[3])
    {
        return Math::rototranslation<T>(
            coord_rt, reinterpret_cast<const T*>(cc.RT), coord);
    }

    /**
     * @brief Wrapper of run with Device.
     * @param coord_rt 
     * @param device 
     * @param coord 
     * @return float* 
     */
    static float* run(float coord_rt[3], const Device& device, const float coord[3])
    {   
        return device.rototranslation(coord_rt, coord);
    }

    /**
     * @brief Vector wrapper of run with CameraCalibration.
     * @tparam T 
     * @param rt
     * @param coord 
     * @return std::vector<T> 
     */
    template <typename T>
    std::vector<T> run(const CameraCalibration<float>& rt, 
        const std::vector<T> coord)
    {
        std::vector<T> ret(std::size_t{3});
        run<T>(ret.data(), cc.RT, coord.data());
        return ret;
    }

    /**
     * @brief Vector wrapper of run with CameraCalibration.
     * @param device
     * @param coord 
     * @return std::vector<float> 
     */
    std::vector<float> run(const Device& device, 
        const std::vector<float> coord)
    {
        std::vector<float> ret(std::size_t{3});
        device.rototranslation(ret.data(), coord.data());
        return ret;
    }
};

} // namespace maeve 

#endif // INTERPOLATION_ROTO_TRANSLATION_HPP
