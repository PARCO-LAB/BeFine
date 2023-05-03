/**
 * @file coco.hpp
 * @date 20/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief TRT_POSE COCO C++ porting header file.
 */

#ifndef NN_TRT_POSE_COCO_HPP
#define NN_TRT_POSE_COCO_HPP


#include <torch/torch.h>

#include <nlohmann/json.hpp>


namespace maeve {

/**
 * @brief 
 * 
 * @tparam T 
 * @param coco_category 
 * @return torch::Tensor 
 */
torch::Tensor coco_category_to_topology(nlohmann::json coco_category);

} // namespace maeve

#endif // NN_TRT_POSE_COCO_HPP
