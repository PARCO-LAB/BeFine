/**
 * @file interpolation_3d.cpp
 * @date 29/08/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Interpolation 3D of 2D keypoints from a depth frame.
 */

#include "maeve/interpolation/interpolation_3d.hpp"


namespace maeve {

MultiHumanKeypoints<float> Interpolation3D::run(
    cv::Size color_mat_size,
    const cv::Mat &depth_mat,
    const MultiHumanKeypoints<float> &humans,
    const CameraCalibration<float>& cc,
    float scale)
{
    MultiHumanKeypoints<float> ret;
    float points2D[2];

    auto color_height = color_mat_size.height;
    auto color_width = color_mat_size.width;
    auto depth_height = depth_mat.rows;
    auto depth_width = depth_mat.cols;

    for (const auto& kp2D: humans)
    {
        SingleHumanKeypoints<float> kp3D;
        for (const auto& e: kp2D)
        { 
            points2D[0] = e.second.at(0) >= color_width ? 
                color_width - 1 : e.second.at(0);
            points2D[1] = e.second.at(1) >= color_height ? 
                color_height - 1 : e.second.at(1);

            int color_map_depth_x = 
                ((points2D[0] / color_width) * depth_width) >= depth_width ? 
                depth_width - 1 : (points2D[0] / color_width) * depth_width;
            int color_map_depth_y = 
                ((points2D[1] / color_height) * depth_height) >= depth_height ? 
                depth_height - 1 : (points2D[1] / color_height) * depth_height;
            
            float d = scale * 
                depth_mat.at<float>(color_map_depth_y, color_map_depth_x);
            
            float points3D[3] = {};
            deproject(points3D, cc, points2D, d);

            kp3D[e.first].push_back(points3D[0] / 1000.0);
            kp3D[e.first].push_back(points3D[1] / 1000.0);
            kp3D[e.first].push_back(points3D[2] / 1000.0);
            kp3D[e.first].push_back(points2D[1]);
            kp3D[e.first].push_back(points2D[0]);
            kp3D[e.first].push_back(d);
            kp3D[e.first].push_back(e.second.at(2));
        }
        ret.push_back(kp3D);
    }

    return ret;
}

MultiHumanKeypoints<float> Interpolation3D::run(
    cv::Size color_mat_size,
    const cv::Mat &depth_mat,
    const MultiHumanKeypoints<float> &humans,
    const Device& device,
    float scale)
{
    MultiHumanKeypoints<float> ret;
    float points2D[2];

    auto color_height = color_mat_size.height;
    auto color_width = color_mat_size.width;
    auto depth_height = depth_mat.rows;
    auto depth_width = depth_mat.cols;

    for (const auto& kp2D: humans)
    {
        SingleHumanKeypoints<float> kp3D;
        for (const auto& e: kp2D)
        { 
            points2D[0] = e.second.at(0) >= color_width ? 
                color_width - 1 : e.second.at(0);
            points2D[1] = e.second.at(1) >= color_height ? 
                color_height - 1 : e.second.at(1);

            int color_map_depth_x = 
                ((points2D[0] / color_width) * depth_width) >= depth_width ? 
                depth_width - 1 : (points2D[0] / color_width) * depth_width;
            int color_map_depth_y = 
                ((points2D[1] / color_height) * depth_height) >= depth_height ? 
                depth_height - 1 : (points2D[1] / color_height) * depth_height;
            
            float d = scale * 
                depth_mat.at<float>(color_map_depth_y, color_map_depth_x);
            
            float points3D[3] = {};
            device.deproject(points3D, points2D, d);

            kp3D[e.first].push_back(points3D[0] / 1000.0);
            kp3D[e.first].push_back(points3D[1] / 1000.0);
            kp3D[e.first].push_back(points3D[2] / 1000.0);
            kp3D[e.first].push_back(points2D[1]);
            kp3D[e.first].push_back(points2D[0]);
            kp3D[e.first].push_back(d);
            kp3D[e.first].push_back(e.second.at(2));
        }
        ret.push_back(kp3D);
    }

    return ret;
}

MultiHumanKeypoints<float> Interpolation3D::run(
    const cv::Mat &color_mat,
    const cv::Mat &depth_mat,
    const MultiHumanKeypoints<float> &kp2D,
    const CameraCalibration<float>& cc,
    float scale)
{
    return Interpolation3D::run(
        color_mat.size(),
        depth_mat,
        kp2D,
        cc,
        scale
    );
}

MultiHumanKeypoints<float> Interpolation3D::run(
    const cv::Mat &color_mat,
    const cv::Mat &depth_mat,
    const MultiHumanKeypoints<float> &kp2D,
    const Device& device,
    float scale)
{
    return Interpolation3D::run(
        color_mat.size(),
        depth_mat,
        kp2D,
        device,
        scale
    );
}

} // namespace maeve
