/**
 * @file zed.hpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief ZED utils header code.
 */

#ifndef CAMERA_ZED_ZED_UTILS_HPP
#define CAMERA_ZED_ZED_UTILS_HPP

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>


namespace maeve {

/**
 * @brief Get OpenCV type from ZED type.
 * @param type ZED type.
 * @return int OpeCV type.
 */
int getOCVtype(sl::MAT_TYPE type) {
    int cv_type = -1;
    switch (type) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        case sl::MAT_TYPE::U16_C1: cv_type = CV_16UC1; break;
        default: break;
    }
    return cv_type;
}

/**
 * @brief Since cv::Mat data requires a uchar* pointer, we get the uchar1 
 * pointer from sl::Mat (getPtr<T>()) cv::Mat and sl::Mat will share a single 
 * memory structure.
 * @param input ZED mat.
 * @return cv::Mat OpenCV mat.
 */
cv::Mat slMat2cvMat(sl::Mat& input) {
    return cv::Mat(input.getHeight(), input.getWidth(), 
        getOCVtype(input.getDataType()), 
        input.getPtr<sl::uchar1>(sl::MEM::CPU), 
        input.getStepBytes(sl::MEM::CPU));
}

/**
 * @brief Since cv::Mat data requires a uchar* pointer, we get the uchar1 
 * pointer from sl::Mat (getPtr<T>()) cv::Mat and sl::Mat will share a single 
 * memory structure.
 * @param input ZED mat.
 * @return cv::cuda::GpuMat OpenCV mat on GPU.
 */
cv::cuda::GpuMat slMat2cvMatGPU(sl::Mat& input) {
    // 
    return cv::cuda::GpuMat(input.getHeight(), input.getWidth(), 
        getOCVtype(input.getDataType()), 
        input.getPtr<sl::uchar1>(sl::MEM::GPU), 
        input.getStepBytes(sl::MEM::GPU));
}

/**
 * @brief Split a string in its parts by delimiter.
 * @param s         String to parse.
 * @param separator Separator inside the string. 
 * @return std::vector<std::string> The vector of string parts. 
 */
std::vector<std::string> split(const std::string& s, char separator) 
{
    std::vector<std::string> output;
    std::string::size_type prev_pos = 0, pos = 0;

    while ((pos = s.find(separator, pos)) != std::string::npos) 
    {
        std::string substring(s.substr(prev_pos, pos - prev_pos));
        output.push_back(substring);
        prev_pos = ++pos;
    }

    output.push_back(s.substr(prev_pos, pos - prev_pos));
    return output;
}

} // namespace maeve

#endif // CAMERA_ZED_ZED_UTILS_HPP
