/**
 * @file dnn.hpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Deep Neural Network loader for pose estimation header code.
 */

#ifndef NN_DNN_HPP
#define NN_DNN_HPP

#if defined(LIB_ALL) && LIB_ALL
#include "maeve/nn/trt_pose/trt_pose_model.hpp"
#include "maeve/nn/open_pose/open_pose_model.hpp"
#else // defined(LIB_ALL) && LIB_ALL

#if defined(TRTPOSE)
#include "maeve/nn/trt_pose/trt_pose_model.hpp"
#elif defined(OPENPOSE)
#include "maeve/nn/open_pose/open_pose_model.hpp"
#else 
#error "LIB_POSE not recognized"
#endif

#endif // defined(LIB_ALL) && LIB_ALL

#include <string>
#include <vector>

namespace maeve {

const size_t HUMAN_NUM = 0;
const std::vector<std::string> HUMAN_PARTS = {
    "nose", "left_eye", "right_eye", "left_ear", "right_ear", 
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow", 
    "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee", 
    "right_knee", "left_ankle", "right_ankle", "neck"
};

#if defined(LIB_ALL) && LIB_ALL
using DNN = TRTPoseModel;
#else // defined(LIB_ALL) && LIB_ALL

#if defined(TRTPOSE)
using DNN = TRTPoseModel;
#elif defined(OPENPOSE)
using DNN = OpenPoseModel;
#else 
#error "LIB_POSE not recognized"
#endif

#endif // defined(LIB_ALL) && LIB_ALL

} // namespace maeve

#endif // NN_DNN_HPP
