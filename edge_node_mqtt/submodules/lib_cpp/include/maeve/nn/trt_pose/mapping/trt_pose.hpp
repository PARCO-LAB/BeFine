/**
 * @file trt_pose.hpp
 * @date 20/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief TRT_POSE library top-level C++ porting header file.
 */

#ifndef NN_TRT_POSE_TRT_POSE_HPP
#define NN_TRT_POSE_TRT_POSE_HPP


#include "coco.hpp"
#include "parse_objects.hpp"
#include "connect_parts.hpp"
#include "paf_score_graph.hpp"
#include "find_peaks.hpp"
#include "refine_peaks.hpp"
#include "munkres.hpp"

#include <iostream>
#include <torch/torch.h>
#include <vector>
#include <tuple>


namespace maeve {

/**
 * @brief 
 * 
 * @param counts 
 * @param peaks 
 * @param input 
 * @param threshold 
 * @param window_size 
 * @param max_count 
 */
void find_peaks_out_torch(torch::Tensor counts, torch::Tensor peaks, 
    torch::Tensor input, const float threshold, const int window_size,
    const int max_count);

/**
 * @brief 
 * 
 * @param input 
 * @param threshold 
 * @param window_size 
 * @param max_count 
 * @return std::vector<torch::Tensor> 
 */
std::vector<torch::Tensor> find_peaks_torch(torch::Tensor input,
    const float threshold, const int window_size, const int max_count);

/**
 * @brief 
 * 
 * @param refined_peaks 
 * @param counts 
 * @param peaks 
 * @param cmap 
 * @param window_size 
 */
void refine_peaks_out_torch(torch::Tensor refined_peaks, torch::Tensor counts,
    torch::Tensor peaks, torch::Tensor cmap, int window_size);

/**
 * @brief 
 * 
 * @param counts 
 * @param peaks 
 * @param cmap 
 * @param window_size 
 * @return torch::Tensor 
 */
torch::Tensor refine_peaks_torch(torch::Tensor counts, torch::Tensor peaks,
    torch::Tensor cmap, int window_size);

/**
 * @brief 
 * 
 * @param score_graph 
 * @param paf 
 * @param topology 
 * @param counts 
 * @param peaks 
 * @param num_integral_samples 
 */
void paf_score_graph_out_torch(torch::Tensor score_graph, torch::Tensor paf, 
    torch::Tensor topology, torch::Tensor counts, torch::Tensor peaks,
    const int num_integral_samples);

/**
 * @brief 
 * 
 * @param paf 
 * @param topology 
 * @param counts 
 * @param peaks 
 * @param num_integral_samples 
 * @return torch::Tensor 
 */
torch::Tensor paf_score_graph_torch(torch::Tensor paf, torch::Tensor topology,
    torch::Tensor counts, torch::Tensor peaks, const int num_integral_samples);

/**
 * @brief 
 * 
 * @param connections 
 * @param score_graph 
 * @param topology 
 * @param counts 
 * @param score_threshold 
 */
void assignment_out_torch(torch::Tensor connections, torch::Tensor score_graph,
    torch::Tensor topology, torch::Tensor counts, const float score_threshold);

/**
 * @brief 
 * 
 * @param score_graph 
 * @param topology 
 * @param counts 
 * @param score_threshold 
 * @return torch::Tensor 
 */
torch::Tensor assignment_torch(torch::Tensor score_graph, 
    torch::Tensor topology, torch::Tensor counts, float score_threshold);

/**
 * @brief 
 * 
 * @param object_counts 
 * @param objects 
 * @param connections 
 * @param topology 
 * @param counts 
 * @param max_count 
 */
void connect_parts_out_torch(torch::Tensor object_counts, 
    torch::Tensor objects, torch::Tensor connections, torch::Tensor topology, 
    torch::Tensor counts, int max_count);

/**
 * @brief 
 * 
 * @param connections 
 * @param topology 
 * @param counts 
 * @param max_count 
 * @return std::vector<torch::Tensor> 
 */
std::vector<torch::Tensor> connect_parts_torch(torch::Tensor connections, 
    torch::Tensor topology, torch::Tensor counts, int max_count);

} // namespace maeve

#endif // NN_TRT_POSE_TRT_POSE_HPP