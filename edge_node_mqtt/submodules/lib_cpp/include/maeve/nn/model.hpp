/**
 * @file device.hpp
 * @date 6/12/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Model header code.
 */

#ifndef NN_MODEL_HPP
#define NN_MODEL_HPP

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>
#include <map>
#include <iostream>

namespace maeve {

template <typename T>
using Coords = std::vector<T>;

template <typename T>
using SingleHumanKeypoints = std::map<std::string, Coords<T>>;

template <typename T>
using MultiHumanKeypoints = std::vector<SingleHumanKeypoints<T>>;

/**
 * @brief Abstract class of DNN models for keypoints extrapolation.
 */
class Model {
public:
    /**
     * @brief Construct a new Model object.
     */
    Model()
        : _to_load{true}
    { }

    /**
     * @brief Copy constructor of a Model object.
     * @param obj Model object reference to copy.
     */
    Model(const Model &obj)
        : _to_load{obj._to_load}
    { }

    /**
     * @brief Destroy the Model object.
     */
    virtual ~Model() 
    { };

    /**
     * @brief Load the model.
     */
    virtual void load() { _to_load = false; }

    /**
     * @brief Call exec method and transform the parsed objects in a vector of 
     * map of keypoints and the 2D coordinates.
     * @param frame         OpenCV color frame.
     * @param to_preprocess Set true if you want to perform preprocessing 
     *                      operations on frame in input.
     * @return MultiHumanKeypoints<float> Vecotr of map of 
     * keypoints and the 2D coordinates.
     */
    virtual MultiHumanKeypoints<float> exec_to_map(
        cv::Mat &frame_mat, 
        bool to_preprocess = true) = 0;

    /**
     * @brief Print Model info. N.B. it has to be loaded.  
     */
    virtual void info() 
    { 
        std::cout << "info() not implemented" << std::endl;
    }

    /**
     * @brief Print Model elapsed time info.
     */
    virtual void elapsed_info() 
    { 
        std::cout << "elapsed_info() not implemented" << std::endl;
    }

protected:
    /// @brief True if the model is not loaded yet.
    bool _to_load;
};

} // namespace maeve

#endif // NN_MODEL_HPP
