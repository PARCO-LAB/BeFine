/**
 * @file parse_objects.hpp
 * @date 20/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief TRT_POSE ParseObjects C++ porting source file.
 */

#include "maeve/nn/trt_pose/mapping/parse_objects.hpp"

#include "maeve/nn/trt_pose/mapping/trt_pose.hpp"


namespace maeve {

ParseObjects::ParseObjects(torch::Tensor topology, float cmap_threshold, 
    float link_threshold, int cmap_window, int line_integral_samples,
    int max_num_parts, int max_num_objects)
    : _topology{topology}
    , _cmap_threshold{cmap_threshold}
    , _link_threshold{link_threshold}
    , _cmap_window{cmap_window}
    , _line_integral_samples{line_integral_samples}
    , _max_num_parts{max_num_parts}
    , _max_num_objects{max_num_objects}
{ }

ParseObjects::~ParseObjects() = default;

std::vector<torch::Tensor> ParseObjects::call(torch::Tensor cmap, 
    torch::Tensor paf)
{
    auto peak = find_peaks_torch(cmap, this->_cmap_threshold, 
        this->_cmap_window, this->_max_num_parts);
    auto peak_counts = peak[0];
    auto peaks = peak[1];

    auto normalized_peaks = refine_peaks_torch(peak_counts, peaks, cmap, 
        this->_cmap_window);
    auto score_graph = paf_score_graph_torch(paf, this->_topology, peak_counts,
        normalized_peaks, this->_line_integral_samples);
    auto connections = assignment_torch(score_graph, this->_topology, 
        peak_counts, this->_link_threshold);
    
    auto object = connect_parts_torch(connections, this->_topology, peak_counts,
        this->_max_num_objects);
    return {object[0], object[1], normalized_peaks};
}

} // namespace maeve