/**
 * @file coco.cpp
 * @date 20/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief TRT_POSE COCO C++ porting source file.
 */


#include "maeve/nn/trt_pose/mapping/coco.hpp"


namespace maeve {

torch::Tensor coco_category_to_topology(nlohmann::json coco_category)
{
    auto skeleton = coco_category["skeleton"];
    const int64_t K = skeleton.size();

    auto options = torch::TensorOptions()
        .dtype(torch::kInt32)
        .layout(torch::kStrided)
        .device(torch::kCPU)
        .requires_grad(false);
    torch::Tensor topology = torch::zeros({K, 4}, options);
    for (int64_t k = 0; k < K; k++)
    {
        topology.index_put_({k, 0}, 2 * k);
        topology.index_put_({k, 1}, (2 * k) + 1);
        topology.index_put_({k, 2}, skeleton[k][0].get<int64_t>() - 1);
        topology.index_put_({k, 3}, skeleton[k][1].get<int64_t>() - 1);
    }
    return topology;
}

} // namespace maeve