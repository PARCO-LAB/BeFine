/**
 * @file filter.cpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Filtering functionalities for RealSense images source code.
 */


#include "maeve/filter/filter.hpp"

#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudawarping.hpp"


namespace maeve {

cv::Mat square(cv::Mat& frame)
{
    const size_t square_side = frame.rows >= frame.cols ? 
        frame.cols : frame.rows;
    const size_t offset_w = (frame.cols - square_side) / 2;
    const size_t offset_h = (frame.rows - square_side) / 2;
    const cv::Rect roi(offset_w, offset_h, square_side, square_side);
    frame = frame(roi);
    return frame;
}

cv::Mat squareup(cv::Mat& frame)
{
    const size_t square_side = frame.rows <= frame.cols ? 
        frame.cols : frame.rows;
    const size_t offset_w = (square_side - frame.cols) / 2;
    const size_t offset_h = (square_side - frame.rows) / 2;
    cv::Mat bigger_image = cv::Mat(cv::Size(square_side,square_side), frame.type(), cv::Scalar(0,0,0));
    cv::Mat roi_of_bigger = bigger_image(cv::Rect(offset_w, offset_h, frame.cols, frame.rows));
    frame.copyTo(roi_of_bigger);
    
    frame = bigger_image;

    
    return frame;
}

cv::Mat resize(cv::Mat& frame, const cv::Size shape)
{
    if (frame.size() == shape)
    {
        return frame;
    }

    cv::cuda::GpuMat frame_d;
    frame_d.upload(frame);

    cv::cuda::resize(frame_d, frame_d, shape);

    frame_d.download(frame);
    return frame;
}

} // namespace maeve
