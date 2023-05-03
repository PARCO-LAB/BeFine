/**
 * @file trt_pose_model.cpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Deep Neural Network model for TRTPose estimation source code.
 */


#include "maeve/nn/trt_pose/trt_pose_model.hpp"

#include "maeve/nn/dnn.hpp"
#include "maeve/filter/filter.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudawarping.hpp"
#include "maeve/nn/trt_pose/mapping/coco.hpp"
#include "maeve/nn/trt_pose/mapping/parse_objects.hpp"

#include <fstream>
#include <cassert>
#include <string.h>
#include <chrono>

namespace maeve {

#if ZC || HY
static bool set_flag = false;
#endif

const static size_t BATCH_SIZE = 1; ///< Batch size for inference.
const static size_t ENGINE_IIDX  = 0;  ///< Engine binding input index.
const static size_t ENGINE_OIDX0 = 1;  ///< Engine binding output 0 index.
const static size_t ENGINE_OIDX1 = 2;  ///< Engine binding output 1 index.

TRTPoseModel::TRTPoseModel(TRTPoseModelKind kind) 
    : Model()
    , _kind{kind}
    , _image_shape_in{}
    , _topology{}
    , _trt_engine{}
    , _trt_context{}
    , _trt_logger{DEBUG_LOG_LEVEL}
    , _in_shape{}
    , _out0_shape{}
    , _out1_shape{}
    , _in_size{}
    , _out0_size{}
    , _out1_size{}
    , _in_ptr_d{}
    , _out0_ptr_d{}
    , _out1_ptr_d{}
    , _out0_buff{}
    , _out1_buff{}
#if 0
    , elapsed_inference{}
    , elapsed_inference_and_copy{}
    , elapsed_exec{}
    , _start{}
    , _stop{}
#endif
    , resize_losing_information{false}
{
    _human_pose_json_fp = fs::path(__FILE__).parent_path() / "models" 
        / HUMAN_POSE_JSON;
}

TRTPoseModel::~TRTPoseModel() 
{
    if (!this->_to_load)
    {
#if ZC
#if defined(HW_CACHE_COHERENCE)
        cudaHostUnregister(this->_out0_buff);
        cudaHostUnregister(this->_out1_buff);
#else // defined(HW_CACHE_COHERENCE)
        cudaFreeHost(this->_in_ptr_d);
        cudaFreeHost(this->_out0_ptr_d);
        cudaFreeHost(this->_out1_ptr_d);
#endif // defined(HW_CACHE_COHERENCE)
#elif HY
#if defined(HW_CACHE_COHERENCE)
        cudaFree(this->_in_ptr_d);
        cudaHostUnregister(this->_out0_buff);
        cudaHostUnregister(this->_out1_buff);
#else // defined(HW_CACHE_COHERENCE)
        cudaFree(this->_in_ptr_d);
        cudaFreeHost(this->_out0_ptr_d);
        cudaFreeHost(this->_out1_ptr_d);
#endif // defined(HW_CACHE_COHERENCE)
#else
        cudaFree(this->_in_ptr_d);
        cudaFree(this->_out0_ptr_d);
        cudaFree(this->_out1_ptr_d);
#endif
        delete[] this->_out0_buff;
        delete[] this->_out1_buff;
#if 0
        float mean_elapsed_exec = std::accumulate(elapsed_exec.begin(), 
            elapsed_exec.end(), 0.0) / elapsed_exec.size();
        float mean_elapsed_infer = std::accumulate(elapsed_inference.begin(), 
            elapsed_inference.end(), 0.0) / elapsed_inference.size();
        float mean_elapsed_infer_and_cpy = std::accumulate(
            elapsed_inference_and_copy.begin(), 
            elapsed_inference_and_copy.end(), 
            0.0) / elapsed_inference_and_copy.size();
        std::streamsize ss = std::cout.precision();
        std::cout << std::setprecision(4) 
            << "TRTPoseModel Average Analysis: " 
            << std::endl  << "elapsed_exec: " 
            << mean_elapsed_exec << " ms"  
            << std::endl << "elapsed_inference: "
            << mean_elapsed_infer << " ms" 
            << std::endl << "elapsed_copy: "
            << mean_elapsed_infer_and_cpy - mean_elapsed_infer << " ms" 
            << std::endl;
        std::cout.precision(ss);
#endif
    }
    //cudaDeviceReset();
}

std::string TRTPoseModel::kind()
{
    switch(this->_kind)
    {
        case TRTPoseModelKind::RESNET:
        {
            return "ResNet";
        }
        case TRTPoseModelKind::DENSENET:
        {
            return "DenseNet";
        }
        default:
        {
            return "";
        }
    }
}

void TRTPoseModel::kind(TRTPoseModelKind kind)
{
    this->_kind = kind;
    this->_to_load = true; 
}

cv::Size TRTPoseModel::get_model_shape_in()
{
    switch(this->_kind)
    {
        case TRTPoseModelKind::RESNET:
        {
            return cv::Size{224, 224};
        }
        case TRTPoseModelKind::DENSENET:
        {
            return cv::Size{256, 256};
        }
        default:
        {
            return cv::Size{};
        }
    }
}

json TRTPoseModel::get_json_human_pose()
{
    std::ifstream human_pose_json_stream{this->_human_pose_json_fp.string()};
    json ret;
    human_pose_json_stream >> ret;
    return ret;
}

void TRTPoseModel::load()
{
    if (!this->_to_load)
    {
        return;
    }

    using namespace nvinfer1;
    fs::path trt_engine_fp;
    switch (this->_kind)
    {
        case TRTPoseModelKind::RESNET:
        {
            trt_engine_fp = fs::path(__FILE__).parent_path() / "models" / CC_DIR
                / RESNET_TRT_ENGINE;
            break;
        }
        case TRTPoseModelKind::DENSENET:
        {
            trt_engine_fp = fs::path(__FILE__).parent_path() / "models" / CC_DIR
                / DENSENET_TRT_ENGINE;
            break;
        }
        default:
        {
            throw std::domain_error("'kind' field not recognized");
        }
    }
    
    // Get topology.
    this->_topology = coco_category_to_topology(this->get_json_human_pose());

    // Load data input shape.
    this->_image_shape_in = this->get_model_shape_in();

    // De-serialize engine from file.
    std::ifstream engineFile(trt_engine_fp.string(), std::ios::binary);
    if (engineFile.fail())
    {
        _trt_logger.log(nvinfer1::ILogger::Severity::kERROR, 
                        "failed to read engine file");
        return;
    }

    engineFile.seekg(0, std::ifstream::end);
    auto fsize = engineFile.tellg();
    engineFile.seekg(0, std::ifstream::beg);

    std::vector<char> engineData(fsize);
    engineFile.read(engineData.data(), fsize);
    engineFile.close();

    // Load engine.
    util::UniquePtr<nvinfer1::IRuntime> runtime{
        nvinfer1::createInferRuntime(_trt_logger.instance())};
    _trt_engine.reset(runtime->deserializeCudaEngine(engineData.data(), fsize, 
        nullptr));
    assert(_trt_engine.get() != nullptr);

    // Create context from engine.
    _trt_context = util::UniquePtr<IExecutionContext>(
        _trt_engine->createExecutionContext());
    if (!_trt_context)
    {
        throw std::runtime_error("context creation failed");
    }

    // Assert datatype.
    assert(_trt_engine->getBindingDataType(ENGINE_IIDX)  == DataType::kFLOAT);
    assert(_trt_engine->getBindingDataType(ENGINE_OIDX0) == DataType::kFLOAT);
    assert(_trt_engine->getBindingDataType(ENGINE_OIDX1) == DataType::kFLOAT);

    // Manage shape.
    auto in_shape   = Dims3{3, _image_shape_in.width, _image_shape_in.height};
    _trt_context->setBindingDimensions(ENGINE_IIDX, in_shape);
    auto out0_shape = _trt_context->getBindingDimensions(ENGINE_OIDX0);
    auto out1_shape = _trt_context->getBindingDimensions(ENGINE_OIDX1);

    this->_out0_shape = std::vector<int64_t>{out0_shape.d, 
        out0_shape.d + out0_shape.nbDims};
    // this->_out0_shape.insert(this->_out0_shape.begin(), BATCH_SIZE);
    this->_out1_shape = std::vector<int64_t>{out1_shape.d, 
        out1_shape.d + out1_shape.nbDims};
    // this->_out1_shape.insert(this->_out1_shape.begin(), BATCH_SIZE);

    // Obtain memory size to allocate
    this->_in_size   = util::get_memory_size(in_shape, sizeof(float));
    this->_out0_size = util::get_memory_size(out0_shape, sizeof(float));
    this->_out1_size = util::get_memory_size(out1_shape, sizeof(float));

    // Allocate output memory.
    this->_out0_buff = new float[this->_out0_size];
    this->_out1_buff = new float[this->_out1_size];

    // Allocate CUDA memory.
#if ZC || HY
    if (!set_flag)
    {
        //cudaSetDeviceFlags(cudaDeviceMapHost);
        set_flag = true;
    }

    // Manage input allocation.
#if HY

#if defined(HW_CACHE_COHERENCE)
    std::cout << "Memory mode: HY (HW Cache Coherence)" << std::endl;
#else // defined(HW_CACHE_COHERENCE)
    std::cout << "Memory mode: HY" << std::endl;
#endif // defined(HW_CACHE_COHERENCE)
    if (cudaMalloc(&this->_in_ptr_d, this->_in_size) != cudaSuccess)
    {
        throw std::runtime_error("input cuda HY memory allocation failed");
    }

#else // HY
    
#if defined(HW_CACHE_COHERENCE)
    std::cout << "Memory mode: ZC (HW Cache Coherence)" << std::endl;
#else // defined(HW_CACHE_COHERENCE)
    std::cout << "Memory mode: ZC" << std::endl;
    float *d_matrix_host_in;
    if (cudaHostAlloc((void **) &d_matrix_host_in, this->_in_size,
            cudaHostAllocMapped) != cudaSuccess)
    {
        throw std::runtime_error("input cuda ZC memory allocation failed");
    }
    else if (cudaHostGetDevicePointer((void **) &this->_in_ptr_d, 
            (void *) d_matrix_host_in, 0) != cudaSuccess)
    {
        throw std::runtime_error("input cuda ZC memory allocation failed");
    }
#endif // defined(HW_CACHE_COHERENCE)

#endif // HY

    // Manage output allocation.
#if defined(HW_CACHE_COHERENCE)
    if (cudaHostRegister(this->_out0_buff, this->_out0_size,
            cudaHostAllocMapped) != cudaSuccess)
    {
        throw std::runtime_error("output 0 cuda ZC memory register failed");
    }
    else if (cudaHostGetDevicePointer((void **) &this->_out0_ptr_d, 
            (void *) this->_out0_buff, 0) != cudaSuccess)
    {
        throw std::runtime_error("output 0 cuda ZC memory register failed");
    }

    if (cudaHostRegister(this->_out1_buff, this->_out1_size,
            cudaHostAllocMapped) != cudaSuccess)
    {
        throw std::runtime_error("output 1 cuda ZC memory register failed");
    }
    else if (cudaHostGetDevicePointer((void **) &this->_out1_ptr_d, 
            (void *) this->_out1_buff, 0) != cudaSuccess)
    {
        throw std::runtime_error("output 1 cuda ZC memory register failed");
    }
#else // defined(HW_CACHE_COHERENCE)
    float *d_matrix_host_out0;
    if (cudaHostAlloc((void **) &d_matrix_host_out0, this->_out0_size,
            cudaHostAllocMapped) != cudaSuccess)
    {
        throw std::runtime_error("output 0 cuda ZC memory allocation failed");
    }
    else if (cudaHostGetDevicePointer((void **) &this->_out0_ptr_d, 
        (void *) d_matrix_host_out0, 0) != cudaSuccess)
    {
        throw std::runtime_error("output 0 cuda ZC memory allocation failed");
    }
    
    float *d_matrix_host_out1;
    if (cudaHostAlloc((void **) &d_matrix_host_out1, this->_out1_size,
            cudaHostAllocMapped) != cudaSuccess)
    {
        throw std::runtime_error("output 1 cuda ZC memory allocation failed");
    }
    else if (cudaHostGetDevicePointer((void **) &this->_out1_ptr_d, 
        (void *) d_matrix_host_out1, 0) != cudaSuccess)
    {
        throw std::runtime_error("output 1 cuda ZC memory allocation failed");
    }
#endif // defined(HW_CACHE_COHERENCE)
#else
    std::cout << "Memory mode: SC" << std::endl;
    if (cudaMalloc((void **)&this->_in_ptr_d, this->_in_size) != cudaSuccess)
    {
        throw std::runtime_error("input cuda SC memory allocation failed");
    }
    if (cudaMalloc((void **)&this->_out0_ptr_d, this->_out0_size) != cudaSuccess)
    {
        throw std::runtime_error("output 0 cuda SC memory allocation failed");
    }
    if (cudaMalloc((void **)&this->_out1_ptr_d, this->_out1_size) != cudaSuccess)
    {
        throw std::runtime_error("output 1 cuda SC memory allocation failed");
    }
#endif

    unsigned int flags;
    cudaGetDeviceFlags(&flags);
    std::cout << "CudaDeviceFlags: 0x" << std::hex << flags << std::dec 
        << std::endl;

#if 0
    cudaEventCreate(&_start);
    cudaEventCreate(&_stop);
#endif

    // Load finished.
    Model::load();
}

void TRTPoseModel::preprocess(cv::Mat &frame_mat, float ** gpu_input)
{
    frame_mat.convertTo(frame_mat, CV_32F, 1.0 / 255);
    cv::cvtColor(frame_mat, frame_mat, cv::COLOR_BGR2RGB);

    const auto mean = cv::Scalar{0.485, 0.456, 0.406};
    const auto std = cv::Scalar{0.229, 0.224, 0.225};

    cv::cuda::GpuMat frame_mat_d;
    frame_mat_d.upload(frame_mat);

    cv::cuda::subtract(frame_mat_d, mean, frame_mat_d, cv::noArray(), -1);
    cv::cuda::divide(frame_mat_d, std, frame_mat_d, 1, -1);

    std::vector< cv::cuda::GpuMat > chw;
    auto input_width = frame_mat.cols;
    auto input_height = frame_mat.rows;
    auto input_size = cv::Size(input_width, input_height);

    for (size_t i = 0; i < 3; ++i)
    {
        chw.emplace_back(cv::cuda::GpuMat(input_size, CV_32FC1, *gpu_input + i * input_width * input_height));
    }
    cv::cuda::split(frame_mat_d, chw);

    // TODO: test the same operation with Torch

    // frame_mat_d.download(frame_mat);
    /*
    auto options = torch::TensorOptions()
        .dtype(torch::kFloat32)
        .device(torch::kCUDA);

    torch::Tensor tensor_image = torch::from_blob(frame_mat.data, 
        { frame_mat.rows, frame_mat.cols, 3 }, options);
    // torch::Tensor tensorImage = torch::from_blob(frame_mat.data, 
    //     { 1, frame_mat.rows, frame_mat.cols, 3 }, torch::kFloat32);
    tensor_image = tensor_image.div(255.0);
    tensor_image = tensor_image.permute({2, 0, 1});

    // Normalize data
    tensor_image[0] = tensor_image[0].sub(0.485).div(0.229);
    tensor_image[1] = tensor_image[1].sub(0.456).div(0.224);
    tensor_image[2] = tensor_image[2].sub(0.406).div(0.225);
    tensor_image.unsqueeze_(0);

    return tensor_image;
    */
}

std::vector<torch::Tensor> TRTPoseModel::exec(cv::Mat &frame_mat, 
    bool to_preprocess)
{
    if (this->_to_load)
    {
        load();
    }

#if 0
    auto start_exec = std::chrono::system_clock::now();
#endif
    if(resize_losing_information) {
        square(frame_mat);
        resize(frame_mat, this->_image_shape_in);
    } else {
        squareup(frame_mat);
        resize(frame_mat, this->_image_shape_in);
    }
    float *in_ptr; (void) in_ptr;
    if (to_preprocess)
    {
        this->preprocess(frame_mat, &this->_in_ptr_d);
        // this->_in_ptr_d = frame_tensor.data_ptr<float>();
        // in_ptr = frame_tensor.to(torch::kCPU).data_ptr<float>();
    }
    // void *in_ptr = frame_mat.data;

    auto start_infer_and_cpy = std::chrono::system_clock::now();
#if HY
    // Copy image data to input binding memory
    // if (cudaMemcpy(this->_in_ptr_d, in_ptr, this->_in_size, 
    //         cudaMemcpyHostToDevice) != cudaSuccess)
    // {
    //     throw std::runtime_error("cuda memory copy of input failed");
    // }

    // Run TensorRT inference
    this->inference();

#if !defined(HW_CACHE_COHERENCE)
    // Copy predictions from output binding memory
    memcpy(_out0_buff, this->_out0_ptr_d, this->_in_size);
    memcpy(_out1_buff, this->_out1_ptr_d, this->_in_size);
#endif // !defined(HW_CACHE_COHERENCE)

#elif ZC
#if defined(HW_CACHE_COHERENCE)
    // if (cudaHostRegister(in_ptr, this->_in_size,
    //         cudaHostAllocMapped) != cudaSuccess)
    // {
    //     throw std::runtime_error("input cuda ZC memory register failed");
    // }
    // else if (cudaHostGetDevicePointer((void **) &this->_in_ptr_d, 
    //         (void *) in_ptr, 0) != cudaSuccess)
    // {
    //     throw std::runtime_error("input cuda ZC memory register failed");
    // }

    // Run TensorRT inference
    this->inference();

    if (cudaHostUnregister(in_ptr) != cudaSuccess)
    {
        throw std::runtime_error("input cuda ZC memory unregister failed");
    }
#else // defined(HW_CACHE_COHERENCE)
    // Copy image data to input binding memory
    // memcpy(this->_in_ptr_d, in_ptr, this->_in_size);

    // Run TensorRT inference
    this->inference();

    // Copy predictions from output binding memory
    memcpy(_out0_buff, this->_out0_ptr_d, this->_in_size);
    memcpy(_out1_buff, this->_out1_ptr_d, this->_in_size);
#endif // defined(HW_CACHE_COHERENCE)
#else
    cudaStream_t stream;
    if (cudaStreamCreate(&stream) != cudaSuccess)
    {
        throw std::runtime_error("cuda stream creation failed");
    }

    // // Copy image data to input binding memory
    // if (cudaMemcpyAsync(this->_in_ptr_d, in_ptr, this->_in_size, 
    //         cudaMemcpyHostToDevice, stream) != cudaSuccess)
    // {
    //     throw std::runtime_error("cuda memory copy of input failed");
    // }

    // Run TensorRT inference
    this->inference(&stream);

    // Copy predictions from output binding memory
    if (cudaMemcpyAsync(_out0_buff, this->_out0_ptr_d, 
            this->_out0_size, cudaMemcpyDeviceToHost, stream) != cudaSuccess)
    {
        throw std::runtime_error("cuda memory copy of output 0 failed");
    }
    if (cudaMemcpyAsync(_out1_buff, this->_out1_ptr_d, 
            this->_out1_size, cudaMemcpyDeviceToHost, stream) != cudaSuccess)
    {
        throw std::runtime_error("cuda memory copy of output 1 failed");
    }
    cudaStreamSynchronize(stream);
    cudaStreamDestroy(stream);
#endif
    auto end_infer_and_cpy = std::chrono::system_clock::now();
#if 0
    elapsed_inference_and_copy.push_back(
        std::chrono::duration<float, std::milli>(
        end_infer_and_cpy - start_infer_and_cpy).count());

    cudaEventSynchronize(_stop);
    float milliseconds = 0;
    cudaEventElapsedTime(&milliseconds, _start, _stop);
    elapsed_inference.push_back(milliseconds);
#endif

    // Parse output.
    auto options = torch::TensorOptions()
        .dtype(torch::kFloat)
        .layout(torch::kStrided)
        .device(torch::kCPU)
        .requires_grad(false);
    
    torch::Tensor out0 = torch::from_blob(_out0_buff, 
        torch::IntArrayRef(this->_out0_shape), options);
    torch::Tensor out1 = torch::from_blob(_out1_buff, 
        torch::IntArrayRef(this->_out1_shape), options);
    auto ret = ParseObjects(this->_topology).call(out0, out1);
#if 0
    auto end_exec = std::chrono::system_clock::now();
    elapsed_exec.push_back(std::chrono::duration<float, std::milli>(
        end_exec - start_exec).count());
#endif
    return ret;
}

MultiHumanKeypoints<float> TRTPoseModel::exec_to_map(
    cv::Mat &frame_mat, 
    bool to_preprocess) 
{
    MultiHumanKeypoints<float> ret;
    auto height = frame_mat.rows;
    auto width = frame_mat.cols;

    // Get DNN model results.
    auto dnn_res = this->exec(frame_mat, to_preprocess);
    auto humans = dnn_res[1];
    auto peaks = dnn_res[2];

    auto humans_amount = humans[0].size(0);
    for (int64_t h = 0; h < humans_amount; ++h)
    {   
        SingleHumanKeypoints<float> kp2D;

        // Get human body keypoints from result.
        auto human = humans[0][h];
        auto num_human_parts = human.size(0);
        int offset = abs(width - height) / 2;

        for (int64_t i = 0; i < num_human_parts; ++i)
        {
            int k = human[i].item<int>();
            if (k >= 0) 
            {
                auto peak = peaks[0][i][k];
                float peak_x = peak[1].item<float>();
                float peak_y = peak[0].item<float>();
                float x = 0;
                float y = 0;
                int resize_factor = 0;
                if (resize_losing_information) 
                {
                    resize_factor = std::min(width, height);
                } 
                else 
                {
                    resize_factor = std::max(width, height);
                }

                x = peak_x * resize_factor;
                y = peak_y * resize_factor;

                if (resize_losing_information) 
                {
                    if(width > height)
                        x = x + offset;
                    else
                        y = y + offset;
                } 
                else 
                {
                    if(width > height)
                        y = y - offset;
                    else
                        x = x - offset;
                }

                if (x >= 0.0 && y >= 0.0) 
                {
                    kp2D[HUMAN_PARTS[i]].push_back(x);
                    kp2D[HUMAN_PARTS[i]].push_back(y);
                    kp2D[HUMAN_PARTS[i]].push_back(100.0); 
                }
            }
        }

        if (kp2D.size() > 0) 
        {
            ret.push_back(kp2D);
        }
    }

    return ret;
}

void TRTPoseModel::info() 
{
    using namespace std;
    auto context = util::UniquePtr<nvinfer1::IExecutionContext>(
        _trt_engine->createExecutionContext());
    
    cout << "----- TRTPoseModel " << kind() << " kind -----" << endl;
    cout << "- Name: " << _trt_engine->getName() << endl;
    auto num_bindings = _trt_engine->getNbBindings();
    cout << "- NbBindings: " << num_bindings << endl;
    for (int i = 0; i < num_bindings; i++)
    {
        cout << "-  - binding " << i << ": { " 
            << "name: " << _trt_engine->getBindingName(i) << ", " 
            << "is_input: " 
                << (_trt_engine->bindingIsInput(i) ? "true" : "false") << ", "
            << "is_shape_binding: " 
                << (_trt_engine->isShapeBinding(i) ? "true" : "false") << ", "
            << "is_execution_binding: " 
                << (_trt_engine->isExecutionBinding(i) ? "true" : "false") 
                << ", "
            << "type: " << static_cast<int>(
                _trt_engine->getBindingDataType(i)) << ", "
            << "format: " << _trt_engine->getBindingFormatDesc(i) << ", "
            << "shape: (";

        auto binding_shape = context->getBindingDimensions(i).nbDims;
        for (int j = 0; j < (binding_shape - 1); j++)
        {
            cout << context->getBindingDimensions(i).d[j] << ", ";
        }
        cout << context->getBindingDimensions(i).d[binding_shape - 1] << ")";

        cout << " } " << endl;
    }

    cout << "- MaxBatchSize: " 
         << _trt_engine->getMaxBatchSize() << endl;
    cout << "- NbLayers: " 
         << _trt_engine->getNbLayers() << endl;
    cout << "- DeviceMemorySize: " 
         << _trt_engine->getDeviceMemorySize() << endl;
    cout << "- NbOptimizationProfiles: " 
         << _trt_engine->getNbOptimizationProfiles() << endl;
    cout << "- isRefittable: " 
         << (_trt_engine->isRefittable() ? "true" : "false") << endl;
    cout << "- hasImplicitBatchDimension: " 
         << (_trt_engine->hasImplicitBatchDimension() ? "true" : "false") 
         << endl;
    
    cout << "-----" << endl;
}

void TRTPoseModel::elapsed_info()
{

}

void TRTPoseModel::inference(cudaStream_t *stream)
{
    bool status;
    void* bindings[] = {this->_in_ptr_d, this->_out0_ptr_d, this->_out1_ptr_d};

#if 0
    cudaEventRecord(_start);
#endif
    if (stream != nullptr)
    {
        status = this->_trt_context->enqueue(BATCH_SIZE, bindings, *stream, 
            nullptr);
    }
    else 
    {
        status = this->_trt_context->execute(BATCH_SIZE, bindings);
    }
#if 0
    cudaEventRecord(_stop);
#endif
    
    if (!status)
    {
        throw std::runtime_error("TensorRT inference failed");
    }
}

} // namespace maeve
