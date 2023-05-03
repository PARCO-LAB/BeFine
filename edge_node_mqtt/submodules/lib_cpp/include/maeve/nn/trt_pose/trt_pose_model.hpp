/**
 * @file trt_pose_model.hpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Deep Neural Network model for TRTPose estimation header code.
 */

#ifndef NN_TRT_POSE_TRT_POSE_MODEL_HPP
#define NN_TRT_POSE_TRT_POSE_MODEL_HPP


#include "maeve/nn/model.hpp"
#include "util.i.hpp"

#include "NvInfer.h"
#include <nlohmann/json.hpp>
#include <torch/torch.h>
#include <cuda_runtime_api.h>
#include "maeve/path.hpp"

#include <tuple>


namespace maeve {

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

namespace fs = std::filesystem;

using json = nlohmann::json;

/// @brief Filename of human pose file in forma JSON.
const std::string HUMAN_POSE_JSON = "human_pose.json";
/// @brief Filename of resnet TensorRT engine.
const std::string RESNET_TRT_ENGINE = 
    "resnet18_baseline_att_224x224_A_epoch_249_trt.engine";
/// @brief Filename of densenet TensorRT engine.
const std::string DENSENET_TRT_ENGINE = 
    "densenet121_baseline_att_256x256_B_epoch_160_trt.engine";

#ifdef COMPUTE_CAPABILITY
const std::string CC_DIR = "CC" STR(COMPUTE_CAPABILITY) "";
#else
const std::string CC_DIR = std::string();
#endif

/**
 * @brief Kind of deep neural network to use.
 */
enum class TRTPoseModelKind 
{
    DENSENET,   ///< densenet121_baseline_att_256x256_B_epoch_160.
    RESNET      ///< resnet18_baseline_att_224x224_A_epoch_249.
};

/**
 * @brief Class for Deep Neural Network pose estimation.
 */
class TRTPoseModel : public Model
{
public:
    /**
     * @brief Construct a new TRTPoseModel object of a specific network kind.
     * @param kind Type of model that you want to use. 
     * Values recognized: TRTPoseModelKind::DENSENET, TRTPoseModelKind::RESNET. 
     * Default: TRTPoseModelKind::DENSENET.
     */
    TRTPoseModel(TRTPoseModelKind kind = TRTPoseModelKind::DENSENET);

    /**
     * @brief Destroy the TRTPoseModel object.
     */
    ~TRTPoseModel() override;

    /**
     * @brief Return the TRTPoseModel kind in string.
     * @return std::string The string that represents the TRTPoseModel kind.
     */
    std::string kind();
    
    /**
     * @brief Set another kind of the TRTPoseModel. 
     * Warning: you have to recall the load method if you call this method.
     * @param kind Type of model that you want to use. 
     * Values recognized: TRTPoseModelKind::DENSENET, TRTPoseModelKind::RESNET.
     */
    void kind(TRTPoseModelKind kind);

    /**
     * @brief Get the shape (a tuple of height and width) of the image to use 
     * as input for the selected pre-trained model. 
     * @return cv::Size Tuple of height and width.
     */
    cv::Size get_model_shape_in();

    /**
     * @brief Get the json human pose object.
     * @return json object.
     */
    json get_json_human_pose();

    /**
     * @brief Load the TRTPoseModel model.
     */
    void load() override;

    /**
     * @brief Preprocessing operations to input frame.
     * The image is processed as a OpenCV Mat.
     * @param frame_mat OpenCV Mat to preprocess.
     */
    void preprocess(cv::Mat &frame_mat, float ** gpu_input);

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
    std::vector<torch::Tensor> exec(
        cv::Mat &frame_mat, 
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
        cv::Mat &frame_mat, 
        bool to_preprocess = true) override;

    /**
     * @brief Print TRTPoseModel info. N.B. it has to be loaded.  
     */
    void info() override;

    /**
     * @brief Print TRTPoseModel elapsed time info.
     */
    void elapsed_info() override;

private:
    /**
     * @brief Perform network inference.
     * @param stream 
     */
    void inference(cudaStream_t *stream = nullptr);

    /// @brief Filepath of human pose JSON file.
    fs::path _human_pose_json_fp;
    /// @brief Type of model that load method will initialize.
    TRTPoseModelKind _kind;
    /// @brief Shape of the input for TRTPoseModel model: (height, width).
    cv::Size _image_shape_in;
    /// @brief Topology of the TRTPoseModel loaded.
    torch::Tensor _topology;
    /// @brief The TensorRT engine used to create the execution context.
    util::UniquePtr<nvinfer1::ICudaEngine> _trt_engine; 
    /// @brief The TensorRT context used to perform inference.
    util::UniquePtr<nvinfer1::IExecutionContext> _trt_context;
    /// @brief The TensorRT logger object.
    util::Logger _trt_logger;

    std::vector<int64_t> _in_shape;   ///< CUDA input data shape.
    std::vector<int64_t> _out0_shape; ///< CUDA output 0 data shape.
    std::vector<int64_t> _out1_shape; ///< CUDA output 0 data shape.

    size_t _in_size;   ///< CUDA input data size.
    size_t _out0_size; ///< CUDA output 0 data size.
    size_t _out1_size; ///< CUDA output 0 data size.
    
    float * _in_ptr_d;   ///< CUDA device pointer of input data.
    float * _out0_ptr_d; ///< CUDA device pointer of output 0 data.
    float * _out1_ptr_d; ///< CUDA device pointer of output 1 data.

    float * _out0_buff; ///< CUDA host pointer of output 0 data. 
    float * _out1_buff; ///< CUDA host pointer of output 1 data. 

#if 0
    // Time analysis structures.
    std::vector<float> elapsed_inference;
    std::vector<float> elapsed_inference_and_copy;
    std::vector<float> elapsed_exec;
    cudaEvent_t _start, _stop;
#endif
    /** 
     * @brief if this one is true, it cut the images to a square. 
     * When false, it augments with black borders.
     */
    bool resize_losing_information;
};

} // namespace maeve

#endif // NN_TRT_POSE_TRT_POSE_MODEL_HPP
