/**
 * @file common.hpp
 * @date 7/12/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Common header of interpolation package.
 */

#ifndef INTERPOLATION_COMMON_HPP
#define INTERPOLATION_COMMON_HPP

namespace maeve 
{ 

/**
 * @brief Camera calibration params.
 */
template <typename T>
struct CameraCalibration 
{  
    int    width;   ///< Width of the image in pixels.
    int    height;  ///< Height of the image in pixels.
    T K[9];         ///< Intrinsics matrix.
    T RT[16];       ///< Extrinsics matrix.
    T ppx;          ///< Horizontal principal point: offset from the left edge.
    T ppy;          ///< Vertical principal point: offset from the left edge.
    T fx;           ///< Focal length: multiple of pixel width.
    T fy;           ///< Focal length: multiple of pixel height.
};

} // namespace maeve 

#endif // INTERPOLATION_COMMON_HPP
