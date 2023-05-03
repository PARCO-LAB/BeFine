/**
 * @file util.i.hpp
 * @date 11/05/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Neural Network package util module.
 */

#ifndef NN_UTIL_I_HPP
#define NN_UTIL_I_HPP


#include "NvInfer.h"

#include <iostream>
#include <memory>
#include <cassert>
#include <numeric>


namespace maeve {

namespace util {

inline size_t get_memory_size(const nvinfer1::Dims& dims, const int32_t elem_size)
{
    return std::accumulate(dims.d, dims.d + dims.nbDims, 1, 
        std::multiplies<int64_t>()) * elem_size;
}

struct InferDeleter
{
    template <typename T>
    void operator()(T* obj) const
    {
        if (obj)
        {
            obj->destroy();
        }
    }
};

template <typename T>
using UniquePtr = std::unique_ptr<T, util::InferDeleter>;

using Severity = nvinfer1::ILogger::Severity;

class Logger : nvinfer1::ILogger 
{
public:
    Logger(Severity severity = Severity::kERROR)
        : _severity{severity}
    {}

    Logger(int severity)
    {
        if (severity > static_cast<int>(Severity::kVERBOSE))
        {
            _severity = Severity::kVERBOSE;
        }
        else 
        {
            _severity = static_cast<Severity>(severity);
        }
    }

    void severity(Severity severity) 
    {
        this->_severity = severity;
    }

    Severity severity() const 
    {
        return _severity;
    }

    void log(Severity severity, const char* msg) noexcept override
    {
        using namespace std;
        if (!(this->should_log(severity))) { return; }

        switch (severity) 
        {
            case Severity::kINTERNAL_ERROR:
            {
                cout << "[FATAL] ";
                break;
            }
            case Severity::kERROR:
            {
                cout << "[ERROR] ";
                break;
            }
            case Severity::kWARNING:
            {
                cout << "[WARN] ";
                break;
            }
            case Severity::kINFO:
            {
                cout << "[INFO]  ";
                break;
            }
            case Severity::kVERBOSE:
            {
                cout << "[VERB]  ";
                break;
            }
            default:
            {
                assert(0);
                break;
            }
        }
        cout << string(msg) << endl;
    }

    nvinfer1::ILogger& instance()
    {
        return *this;
    }

private:
    bool should_log(Severity severity) 
    {
#if DEBUG
        return severity <= this->_severity;
#else
        return false;
#endif
    }

    Severity _severity;
};

} // namespace util

} // namespace maeve

#endif // NN_UTIL_I_HPP
