/**
 * @file parse_objects.hpp
 * @date 20/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief TRT_POSE ParseObjects C++ porting header file.
 */

#ifndef NN_TRT_POSE_PARSE_OBJECTS_HPP
#define NN_TRT_POSE_PARSE_OBJECTS_HPP


#include <torch/torch.h>

#include <iostream>


namespace maeve {

/**
 * @brief 
 * 
 */
class ParseObjects
{
public:
    /**
     * @brief Construct a new Parse Objects object
     * 
     * @param topology 
     * @param cmap_threshold 
     * @param link_threshold 
     * @param cmap_window 
     * @param line_integral_samples 
     * @param max_num_parts 
     * @param max_num_objects 
     */
    ParseObjects(torch::Tensor topology, float cmap_threshold = 0.1, 
        float link_threshold = 0.1, int cmap_window = 5, 
        int line_integral_samples  = 7, int max_num_parts = 100, 
        int max_num_objects = 100);

    /**
     * @brief Destroy the Parse Objects object
     * 
     */
    ~ParseObjects();

    /**
     * @brief 
     * 
     * @param cmap 
     * @param paf 
     * @return std::vector<torch::Tensor> 
     */
    std::vector<torch::Tensor> call(torch::Tensor cmap, torch::Tensor paf); 
private:
    /// @brief
    torch::Tensor _topology;
    /// @brief
    float _cmap_threshold;
    /// @brief
    float _link_threshold;
    /// @brief
    int _cmap_window;
    /// @brief
    int _line_integral_samples;
    /// @brief
    int _max_num_parts;
    /// @brief
    int _max_num_objects;
};

} // namespace maeve

#endif // NN_TRT_POSE_PARSE_OBJECTS_HPP