/**
 * @file open_pose_model.hpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Deep Neural Network model for OpenPose estimation header code.
 */

#ifndef NN_OPEN_POSE_OPEN_POSE_MODEL_HPP
#define NN_OPEN_POSE_OPEN_POSE_MODEL_HPP

#include "maeve/nn/model.hpp"

#include <torch/torch.h>
#include <cuda_runtime_api.h>

#include <openpose/headers.hpp>

namespace maeve {

/**
 * @brief Class for Deep Neural Network pose estimation.
 */
class OpenPoseModel : public Model
{
public:
    /**
     * @brief Construct a new OpenPoseModel object of a specific network kind.
     */
    OpenPoseModel();

    /**
     * @brief Get the shape (a tuple of height and width) of the image to use 
     * as input for the selected pre-trained model. 
     * @return cv::Size Tuple of height and width.
     */
    cv::Size get_model_shape_in();

    /**
     * @brief Load the OpenPoseModel model.
     */
    void load() override;

    /**
     * @brief Preprocessing operations to input frame.
     * The image is processed as a OpenCV Mat.
     * @param frame_mat OpenCV Mat to preprocess.
     */
    void preprocess(cv::Mat &frame_mat);

    /**
     * @brief  Get the keypoints human pose from the image frame in input 
     * using the model loaded. 
     * Warning: you have to call the load method before this one.
     * 
     * @param frame         OpenCV color frame.
     * @param to_preprocess Set true if you want to perform preprocessing 
     *                      operations on frame in input.
     * @return std::vector<torch::Tensor> Vector of parsered results.
     */
    std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> exec(
        cv::Mat &frame, 
        bool to_preprocess = true);

    /**
     * @brief Call exec method and transform the parsed objects in a map of 
     * keypoints and the 2D coordinates.
     * @param frame         OpenCV color frame.
     * @param to_preprocess Set true if you want to perform preprocessing 
     *                      operations on frame in input.
     * @return MultiHumanKeypoints<float> Map of 
     * keypoints and the 2D coordinates.
     */
    MultiHumanKeypoints<float> exec_to_map(
        cv::Mat &frame, 
        bool to_preprocess = true) override;

    /**
     * @brief Print OpenPoseModel info. N.B. it has to be loaded.  
     */
    void info() override;

    /**
     * @brief Print OpenPoseModel elapsed time info.
     */
    void elapsed_info() override;

private:
    op::Wrapper _op_wrapper;
    op::PoseModel _kind;

    /// @brief Shape of the input for OpenPose model: (height, width).
    cv::Size _image_shape_in;
};

} // namespace maeve

#endif // NN_OPEN_POSE_OPEN_POSE_MODEL_HPP