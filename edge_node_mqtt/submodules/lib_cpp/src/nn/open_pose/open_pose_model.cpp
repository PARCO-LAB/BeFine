/**
 * @file open_pose_model.cpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Deep Neural Network model for OpenPose estimation source code.
 */

#include "maeve/nn/open_pose/open_pose_model.hpp"

#include "maeve/nn/dnn.hpp"
#include "maeve/filter/filter.hpp"

#define OPENPOSE_FLAGS_DISABLE_PRODUCER
#define OPENPOSE_FLAGS_DISABLE_DISPLAY
#include <openpose/flags.hpp>
#include <openpose/pose/poseParameters.hpp>

DEFINE_bool(no_display, true,
            "Enable to disable the visual display.");

namespace maeve {

OpenPoseModel::OpenPoseModel()
    : Model()
    , _op_wrapper{op::ThreadManagerMode::Asynchronous}
    , _kind{op::PoseModel::MPI_15_4}
    , _image_shape_in{}
{ 
    FLAGS_face = false;
    FLAGS_hand = false;
    FLAGS_model_folder = "../../openpose/models";
    FLAGS_model_pose = "MPI_4_layers";
    FLAGS_net_resolution = "224x224";
}

cv::Size OpenPoseModel::get_model_shape_in()
{
    switch(this->_kind)
    {
        default:
        {
            return cv::Size{224, 224};
        }
    }
}

void OpenPoseModel::load()
{
    if (!this->_to_load)
    {
        return;
    }

    // Load data input shape.
    this->_image_shape_in = this->get_model_shape_in();

    try
    {
        // Configuring OpenPose

        // logging_level
        op::checkBool(
            0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
            __LINE__, __FUNCTION__, __FILE__);
        op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
        op::Profiler::setDefaultX(FLAGS_profile_speed);

        // Applying user defined configuration - GFlags to program variables
        // outputSize
        const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
        // netInputSize
        const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
        // faceNetInputSize
        const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
        // handNetInputSize
        const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
        // poseMode
        const auto poseMode = op::flagsToPoseMode(FLAGS_body);
        // poseModel
        const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
        // JSON saving
        if (!FLAGS_write_keypoint.empty())
            op::opLog(
                "Flag `write_keypoint` is deprecated and will eventually be removed. Please, use `write_json`"
                " instead.", op::Priority::Max);
        // keypointScaleMode
        const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
        // heatmaps to add
        const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
                                                      FLAGS_heatmaps_add_PAFs);
        const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
        // >1 camera view?
        const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
        // Face and hand detectors
        const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
        const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
        // Enabling Google Logging
        const bool enableGoogleLogging = true;

        // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
        const op::WrapperStructPose wrapperStructPose{
            poseMode, netInputSize, FLAGS_net_resolution_dynamic, outputSize, keypointScaleMode, FLAGS_num_gpu,
            FLAGS_num_gpu_start, FLAGS_scale_number, (float)FLAGS_scale_gap,
            op::flagsToRenderMode(FLAGS_render_pose, multipleView), poseModel, !FLAGS_disable_blending,
            (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap, FLAGS_part_to_show, op::String(FLAGS_model_folder),
            heatMapTypes, heatMapScaleMode, FLAGS_part_candidates, (float)FLAGS_render_threshold,
            FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max, op::String(FLAGS_prototxt_path),
            op::String(FLAGS_caffemodel_path), (float)FLAGS_upsampling_ratio, enableGoogleLogging};
        _op_wrapper.configure(wrapperStructPose);
        // Face configuration (use op::WrapperStructFace{} to disable it)
        const op::WrapperStructFace wrapperStructFace{
            FLAGS_face, faceDetector, faceNetInputSize,
            op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
            (float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold};
        _op_wrapper.configure(wrapperStructFace);
        // Hand configuration (use op::WrapperStructHand{} to disable it)
        const op::WrapperStructHand wrapperStructHand{
            FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
            op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
            (float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold};
        _op_wrapper.configure(wrapperStructHand);
        // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
        const op::WrapperStructExtra wrapperStructExtra{
            FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads};
        _op_wrapper.configure(wrapperStructExtra);
        // Output (comment or use default argument to disable any output)
        const op::WrapperStructOutput wrapperStructOutput{
            FLAGS_cli_verbose, op::String(FLAGS_write_keypoint), op::stringToDataFormat(FLAGS_write_keypoint_format),
            op::String(FLAGS_write_json), op::String(FLAGS_write_coco_json), FLAGS_write_coco_json_variants,
            FLAGS_write_coco_json_variant, op::String(FLAGS_write_images), op::String(FLAGS_write_images_format),
            op::String(FLAGS_write_video), FLAGS_write_video_fps, FLAGS_write_video_with_audio,
            op::String(FLAGS_write_heatmaps), op::String(FLAGS_write_heatmaps_format), op::String(FLAGS_write_video_3d),
            op::String(FLAGS_write_video_adam), op::String(FLAGS_write_bvh), op::String(FLAGS_udp_host),
            op::String(FLAGS_udp_port)};
        _op_wrapper.configure(wrapperStructOutput);
        // No GUI. Equivalent to: _op_wrapper.configure(op::WrapperStructGui{});
        // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
        if (FLAGS_disable_multi_thread)
            _op_wrapper.disableMultiThreading();
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }

    // Starting OpenPose
    op::opLog("Starting OpenPose", op::Priority::High);
    _op_wrapper.start();

    // Load finished.
    Model::load();
}

void OpenPoseModel::preprocess(cv::Mat &frame_mat)
{
    cv::cvtColor(frame_mat, frame_mat, cv::COLOR_BGR2RGB);
}

std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> OpenPoseModel::exec(
    cv::Mat &frame, 
    bool to_preprocess)
{
    if (this->_to_load)
    {
        load();
    }

    square(frame);
    resize(frame, this->_image_shape_in);
    if (to_preprocess)
    {
        this->preprocess(frame);
    }
    const op::Matrix frame_to_process = OP_CV2OPCONSTMAT(frame);

    auto datum = _op_wrapper.emplaceAndPop(frame_to_process);
    return datum;
}

MultiHumanKeypoints<float> OpenPoseModel::exec_to_map(
    cv::Mat &frame, 
    bool to_preprocess)
{
    MultiHumanKeypoints<float> ret;
    auto datum = this->exec(frame, to_preprocess);

    try
    {
        // Example: How to use the pose keypoints
        if (datum != nullptr && !datum->empty())
        {
            for (size_t h = 0; h < datum->size(); ++h)
            {
                SingleHumanKeypoints<float> kp2D;

                const auto &pose_kp = datum->at(h)->poseKeypoints;
                const auto num_people     = pose_kp.getSize(0);
                const auto num_body_parts = pose_kp.getSize(1);
                const auto num_coords     = pose_kp.getSize(2);

                // Get person with maxscore.
                std::vector<double> scores(num_people, 0);
                for (auto person = 0; person < num_people; person++) {
                    for (auto bp = 0; bp < num_body_parts; bp++)
                    {
                        scores[person] += pose_kp[{person, bp, 2}];      
                    }
                }
                auto person = std::distance(scores.begin(), 
                    std::max_element(scores.begin(), scores.end()));

                for (auto bp = 0; bp < num_body_parts; bp++) {
                    const auto baseIndex = num_coords 
                        * (person * num_body_parts + bp);
                    std::string kp = op::getPoseBodyPartMapping(
                        op::PoseModel::COCO_18).at(bp);
                    kp2D[kp].push_back(pose_kp[baseIndex]);
                    kp2D[kp].push_back(pose_kp[baseIndex + 1]);
                    kp2D[kp].push_back(pose_kp[baseIndex + 2]); 
                }

                ret.push_back(kp2D);
            }
        }
        else 
        {
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
        }
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }

    return ret;
}

void OpenPoseModel::info()
{

}

void OpenPoseModel::elapsed_info()
{
    
}

} // namespace maeve
