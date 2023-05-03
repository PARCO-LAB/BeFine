/**
 * @file filter.hpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Filtering functionalities for RealSense images header code.
 */

#ifndef FILTER_FILTER_HPP
#define FILTER_FILTER_HPP


#include <opencv2/opencv.hpp>


namespace maeve {

/**
 * @brief Makes an image squared and return it. 
 * The image is processed as a OpenCV Mat.
 * @param frame OpenCV Mat pointer that will be squared.
 * @return cv::Mat* The same pointer passed as parameter. 
 */
cv::Mat square(cv::Mat& frame);

/**
 * @brief Makes an image squared with black border and return it. 
 * The image is processed as a OpenCV Mat.
 * @param frame OpenCV Mat pointer that will be squared.
 * @return cv::Mat* The same pointer passed as parameter. 
 */
cv::Mat squareup(cv::Mat& frame);

/**
 * @brief Transform an image in a target shape and return it. 
 * The image is processed as a OpenCV Mat in CUDA.
 * @param frame OpenCV Mat pointer that will be squared.
 * @return cv::Mat* The same pointer passed as parameter. 
 */
cv::Mat resize(cv::Mat& frame, const cv::Size shape);

} // namespace maeve

#endif // FILTER_FILTER_HPP
