/**
 * @file math.hpp
 * @date 5/12/2021
 * @author Mirco De Marchi (mirco@demarchi.dev)
 * @brief Maeve math functionalities.
 */

#ifndef MATH_HPP
#define MATH_HPP

#include <cmath>
#include <functional>
#include <cassert>
#include <stdexcept>
#include <tuple>
#include <algorithm>
#include <iterator>
#include <limits>
#include <vector>
#include <iostream>


namespace maeve {

class Math 
{
public:
    /**
     * \brief Element wise multiplication between two arrays.
     * \tparam T     Type of each source and destination elements.
     * \param dst    Array to write the result.
     * \param src1   First operand array.
     * \param src2   Second operand array.
     * \param length Length of the arrays.
     * \return T* The destination array pointer.
     */
    template <typename T>
    static T* arr_mul(T* dst, const T* src1, const T* src2, std::size_t length)
    {
        for (size_t i = 0; i < length; ++i)
        {
            dst[i] = src1[i] * src2[i];
        }
        return dst;
    }

    template <typename T>
    static std::vector<T> arr_mul(
        const std::vector<T>& src1, const std::vector<T>& src2)
    {
        std::vector<T> ret;
        std::size_t length = std::min(src1.size(), src2.size());
        ret.resize(length);
        arr_mul<T>(ret.data(), src1.data(), src2.data(), length);
        return ret;
    }

    /**
     * \brief Element wise summation between two arrays.
     * \tparam T     Type of each source and destination elements.
     * \param dst    Array to write the result.
     * \param src1   First operand array.
     * \param src2   Second operand array.
     * \param length Length of the arrays.
     * \return T* The destination array pointer.
     */
    template <typename T>
    static T* arr_sum(T* dst, const T* src1, const T* src2, std::size_t length)
    {
        for (std::size_t i = 0; i < length; ++i)
        {
            dst[i] = src1[i] + src2[i];
        }
        return dst;
    }

    template <typename T>
    static std::vector<T> arr_sum(
        const std::vector<T>& src1, const std::vector<T>& src2)
    {
        std::vector<T> ret;
        std::size_t length = std::min(src1.size(), src2.size());
        ret.resize(length);
        arr_sum<T>(ret.data(), src1.data(), src2.data(), length);
        return ret;
    }

    /**
     * \brief Summation of value to elements of array.
     * \tparam T     Type of each source and destination elements.
     * \param dst    Array to write the result.
     * \param src1   First operand array.
     * \param src2   Second operand array.
     * \param length Length of the arrays.
     * \return T* The destination array pointer.
     */
    template <typename T>
    static T* arrval_sum(T* dst, const T* src1, T src2, std::size_t length)
    {
        for (std::size_t i = 0; i < length; ++i)
        {
            dst[i] = src1[i] + src2;
        }
        return dst;
    }

    template <typename T>
    static std::vector<T> arrval_sum(
        const std::vector<T>& src1, T src2)
    {
        std::vector<T> ret;
        std::size_t length = src1.size();
        ret.resize(length);
        arrval_sum<T>(ret.data(), src1.data(), src2, length);
        return ret;
    }

    /**
     * \brief Multiplication of value to elements of array.
     * \tparam T     Type of each source and destination elements.
     * \param dst    Array to write the result.
     * \param src1   First operand array.
     * \param src2   Second operand array.
     * \param length Length of the arrays.
     * \return T* The destination array pointer.
     */
    template <typename T>
    static T* arrval_mul(T* dst, const T* src1, T src2, std::size_t length)
    {
        for (std::size_t i = 0; i < length; ++i)
        {
            dst[i] = src1[i] * src2;
        }
        return dst;
    }

    template <typename T>
    static std::vector<T> arrval_mul(
        const std::vector<T>& src1, T src2)
    {
        std::vector<T> ret;
        std::size_t length = std::min(src1.size(), src2.size());
        ret.resize(length);
        arrval_mul<T>(ret.data(), src1.data(), src2, length);
        return ret;
    }

    /**
     * @brief Compute mean between two arrays. 
     * @tparam T 
     * @param dst    Array to write the result.
     * @param src1   First operand array.
     * @param src2   Second operand array.
     * @param length Length of the arrays.
     * @return T* The destination array pointer.
     */
    template <typename T>
    static T* arr_mean(T* dst, const T* src1, const T* src2, std::size_t length)
    {
        for (std::size_t i = 0; i < length; ++i)
        {
            dst[i] = (src1[i] + src2[i]) / 2.0;
        }
        return dst;
    }

    template <typename T>
    static std::vector<T> arr_mean(
        const std::vector<T>& src1, const std::vector<T>& src2)
    {
        std::size_t length = std::min(src1.size(), src2.size());
        std::vector<T> ret;
        ret.resize(length);
        arr_mean<T>(ret.data(), src1.data(), src2.data(), length);
        return ret;
    }

    /**
     * @brief Get the index in a vector of an element contained. 
     * If the element is not contained in the vector return -1.
     * @tparam T 
     * @param src  Source array.
     * @param elem Element to find.
     * @return std::int64_t -1 if not find. 
     */
    template <typename T>
    static std::int64_t idx(const std::vector<T>& src, T elem)
    {
        auto it = std::find(src.begin(), src.end(), elem);
        if (it != src.end())
        {
            return it - src.begin();
        }
        return -1;
    }

    template <typename T>
    static std::vector<std::int64_t> idx(
        const std::vector<T>& src, const std::vector<T>& elements)
    {
        std::vector<std::int64_t> ret;
        ret.resize(elements.size());
        for (std::size_t i = 0; i < ret.size(); ++i)
        {
            ret[i] = idx<T>(src, elements[i]);
        }
        return ret;
    }

    /**
     * @brief Check if the array contains or not an element.
     * @tparam T 
     * @param src  Source array.
     * @param elem Element to find.
     * @return true  If contains the element.
     * @return false If not contains the element.
     */
    template <typename T>
    static bool contains(const std::vector<T>& src, T elem)
    {
        auto it = std::find(src.begin(), src.end(), elem);
        if (it != src.end())
        {
            return true;
        }
        return false;
    }

    template <typename T>
    static std::vector<bool> contains(
        const std::vector<T>& src, const std::vector<T>& elements)
    {
        std::vector<bool> ret;
        ret.resize(elements.size());
        for (std::size_t i = 0; i < ret.size(); ++i)
        {
            ret[i] = contains<T>(src, elements[i]);
        }
        return ret;
    }

    /**
     * \brief Multiplication between a matrix and an array.
     * Used for y = Wx
     * @tparam T      Type of each source and destination elements.
     * \param arr_dst Array destination to write the result.
     * \param mat_src Matrix source, left operand.
     * \param arr_src Array source, right operand.
     * \param rows    Amount of rows.
     * \param cols    Amount of columns.
     * \return T* The destination array pointer.
     */
    template <typename T>
    static T* matarr_mul(T* arr_dst, const T* mat_src, const T* arr_src, 
        std::size_t rows, std::size_t cols)
    {
        if (arr_src == arr_dst) 
        {
            throw std::runtime_error("arr_src, arr_dst have to be different "
                                     "in order to perform matarr_mul");
        }

        for (std::size_t i = 0; i < rows; ++i)
        {
            arr_dst[i] = T{0};
            for (std::size_t j = 0; j < cols; ++j)
            {
                arr_dst[i] += mat_src[(i * cols) + j] * arr_src[j];
            }
        }
        return arr_dst;
    }

    template <typename T>
    static std::vector<T> matarr_mul(
        const std::vector<T>& mat_src, const std::vector<T>& arr_src, 
        std::size_t rows)
    {
        std::vector<T> ret;
        ret.resize(rows);
        matarr_mul<T>(ret.data(), mat_src.data(), arr_src.data(), 
            rows, arr_src.size());
        return ret;
    }

    /**
     * @brief Calculate the transpose of a matrix. 
     * @tparam T 
     * @param mat_dst Matrix destination: COLS x ROWS.
     * @param mat_src Matrix source: ROWS x COLS.
     * @param rows    Number of rows of the source matrix. 
     * @param cols    Number of columns of the source matrix.
     * @return T* The pointer to the matrix destination.
     */
    template <typename T>
    static T* transpose(T* mat_dst, const T* mat_src, 
        std::size_t rows, std::size_t cols)
    {
        const T* mat = mat_src;
        if (mat_dst == mat_src)
        {
            T* mat_tmp = new T[rows * cols];
            std::copy(mat_src, mat_src + (rows * cols), mat_tmp);
            mat = mat_tmp;
        }

        for (std::size_t r = 0; r < rows; ++r)
        {
            for (std::size_t c = 0; c < cols; ++c)
            {
                mat_dst[c * rows + r] = mat[r * cols + c];
            }
        }

        if (mat_dst == mat_src)
        {
            delete[] mat;
        }

        return mat_dst;
    }

    template <typename T>
    static std::vector<T> transpose(const std::vector<T>& mat_src, 
        std::size_t rows, std::size_t cols)
    {
        std::vector<T> ret(std::size_t(rows * cols));
        transpose(ret.data(), mat_src.data(), rows, cols);
        return ret;
    }

    /**
     * @brief Determinant of a matrix.
     * 
     * Determinant property:
     * The determinant of upper triangular matrix is the product of all 
     * diagonal elements. 
     * 
     * Algorithm steps:
     * 1) Convert the matrix in the upper-triangular form.
     * 2) Computes the product of the diagonal elements.
     * 
     * @tparam T 
     * @param mat_src Source matrix: SIDE x SIDE.
     * @param side Shape of the squared matrix.
     * @return T The determinant value.
     */
    template <typename T>
    static T det(const T* mat_src, std::size_t side)
    {
        if (side == 1)
        {
            return mat_src[0];
        }
        if (side == 2)
        {
            return (mat_src[0] * mat_src[3]) - (mat_src[2] * mat_src[1]);
        }
        else if (side == 3)
        {
            return (mat_src[0] 
                    * ((mat_src[4] * mat_src[8]) - (mat_src[7] * mat_src[5])))
                - (mat_src[1] 
                    * ((mat_src[3] * mat_src[8]) - (mat_src[6] * mat_src[5])))
                + (mat_src[2] 
                    * ((mat_src[3] * mat_src[7]) - (mat_src[6] * mat_src[4])));
        }

        T det{1}; 
        T total{1};
    
        // Temporary array for storing row.
        T *tmp = new T[side + 1];
        T *mat = new T[side * side];
        std::copy(mat_src, mat_src + (side * side), mat);
    
        // Gaussian elimination.
        for (std::size_t col = 0; col < side; ++col)
        {
            std::size_t row = col; //< Start from the diagonal element. 
    
            // Find if the row has a non-zero value.
            while (mat[row * side + col] == 0 && row < side) row++;
            
            // If not then the determinant is zero.
            if (row == side) 
            {
                delete[] tmp;
                return T{0};
            }
            // The row has a non-zero value.
            if (row != col)
            {
                // Swap the rows.
                for (std::size_t j = 0; j < side; ++j)
                {
                    std::swap(mat[row * side + j], mat[col * side + j]);
                }
                // Determinant sign changes when we shift rows.
                det *= std::pow(-1, row - col);
            }
    
            // Storing the values of diagonal row elements.
            for (std::size_t j = 0; j < side; ++j)
            {
                tmp[j] = mat[col * side + j];
            }

            // Traversing every row below the diagonal element.
            T num1 = tmp[col]; //< Value of diagonal element.
            for (std::size_t i = col + 1; i < side; ++i)
            {
                T num2 = mat[i * side + col]; //< Value of next row element.
    
                // Traversing every element of row.
                for (std::size_t k = col + 1; k < side; ++k)
                {
                    /* 
                     * Multiplying to make the diagonal element and next row 
                     * element equal.
                     */
                    mat[i * side + k] 
                        = (num1 * mat[i * side + k]) - (num2 * tmp[k]);

                    // Alternative without total.
                    // mat[i * side + k] -= num2 * tmp[k] / num1;
                }
                total = total * num1; //< Det(k * A) = k * Det(A).
            }
        }
    
        // Multiply the diagonal elements to get determinant.
        for (std::size_t i = 0; i < side; ++i)
        {
            det *= mat[i * side + i];
        }

        delete[] tmp;
        delete[] mat;
        return det / total; //< Det(k * A) / k 
    }

    template <typename T>
    static T det(const std::vector<T>& mat_src, std::size_t side)
    {
        return det(mat_src.data(), side);
    }

    /**
     * @brief Calculate the cofactor of a matrix.
     * @tparam T 
     * @param mat_dst Destination matrix: SIDE x SIDE.
     * @param mat_src Source matrix: SIDE x SIDE.
     * @param side    Shape of the squared matrix.
     * @return T* The pointer to the matrix destination.
     */
    template <typename T>
    static T* comatrix(T* mat_dst, const T* mat_src, 
        std::size_t side)
    {
        T* cofactor_mat = new T[(side - 1) * (side - 1)];
        const T* mat = mat_src;
        if (mat_dst == mat_src)
        {
            T* mat_tmp = new T[side * side];
            std::copy(mat_src, mat_src + (side * side), mat_tmp);
            mat = mat_tmp;
        }

        for (std::size_t i = 0; i < side * side; ++i)
        {
            std::size_t cofactor_row = i / side;
            std::size_t cofactor_col = i % side;
            std::size_t cofactor_idx = 0;

            // Make cofactor matrix of cofactor_row and cofactor_col.
            for (std::size_t row = 0; row < side; ++row)
            {
                if (row == cofactor_row) continue;
                for (std::size_t col = 0; col < side; ++col)
                {
                    if (col == cofactor_col) continue;
                    cofactor_mat[cofactor_idx] = mat[row * side + col];
                    cofactor_idx++;
                }
            }

            // Calculate determinant of cofactor matrix.
            mat_dst[i] = det<T>(cofactor_mat, side - 1) 
                * std::pow(-1, cofactor_row + cofactor_col);
        }

        if (mat_dst == mat_src)
        {
            delete[] mat;
        }
        delete[] cofactor_mat;
        return mat_dst;
    }

    template <typename T>
    static std::vector<T> comatrix(const std::vector<T>& mat_src, 
        std::size_t side)
    {
        std::vector<T> ret(std::size_t(side * side));
        comatrix(ret.data(), mat_src.data(), side);
        return ret;
    }

    /**
     * @brief Calculate the inverse of a matrix.
     * @tparam T 
     * @param mat_dst Destination matrix: SIDE x SIDE.
     * @param mat_src Source matrix: SIDE x SIDE.
     * @param side    Shape of the squared matrix.
     * @return T* The pointer to the matrix destination.
     * Return the nullptr if the matrix is not invertible.
     */
    template <typename T>
    static T* inv(T* mat_dst, const T* mat_src, 
        std::size_t side)
    {
        T mat_det = det<T>(mat_src, side);
        if (mat_det == T{0}) 
        {
            return nullptr;
        }

        comatrix<T>(mat_dst, mat_src, side);
        transpose<T>(mat_dst, mat_dst, side, side);
        arrval_mul<T>(mat_dst, mat_dst, T{1} / mat_det, side * side);
        
        return mat_dst;
    }

    template <typename T>
    static std::vector<T> inv(const std::vector<T>& mat_src, 
        std::size_t side)
    {
        std::vector<T> ret(std::size_t(side * side));
        inv(ret.data(), mat_src.data(), side);
        return ret;
    }

    /**
     * @brief Rototranslation of the world point according to the
     * extrinsics parameters of the camera.
     * @tparam T 
     * @param coord_rt Destination pointer of rototraslated 3D coordinates.
     * @param rt       Matrix 4x4 of camera extrinsics.
     * @param coord    Source of the 3D coordinates.
     * @return T* The destination pointer.
     */
    template <typename T>
    static T* rototranslation(T coord_rt[3], const T rt[16], const T coord[3])
    {   
        const std::size_t SIDE(4);
        T coord_ext[SIDE] = {coord[0], coord[1], coord[2], T(1)};
        T ret[SIDE];

        matarr_mul<T>(ret, rt, coord_ext, SIDE, SIDE);
        
        coord_rt[0] = ret[0]; 
        coord_rt[1] = ret[1]; 
        coord_rt[2] = ret[2];
        return coord_rt;
    }

    /**
     * @brief Vector wrapper of rototranslation.
     * @tparam T 
     * @param rt
     * @param coord 
     * @return std::vector<T> 
     */
    template <typename T>
    std::vector<T> rototranslation(const std::vector<T> rt, 
        const std::vector<T> coord)
    {
        std::vector<T> ret(std::size_t{3});
        rototranslation<T>(ret.data(), rt.data(), coord.data());
        return ret;
    }

    /**
     * @brief Deprojection of 2D coordinates to 3D coordinates according to
     * a intrinsic k matrix.
     * @tparam T 
     * @param coord_3d Destination of the 3D coordinates.
     * @param k        Matrix 3x3 of camera intrinsics.
     * @param coord    Source of the 2D coordinates.
     * @param d        Depth value in coord.
     * @return T* The destination pointer.
     */
    template <typename T>
    static T* deproject(T coord_3d[3], const T k[9], const T coord[2], T d)
    {   
        const std::size_t SIDE(3);
        T coord_ext[SIDE] = {coord[0], coord[1], T{1}};
        T k_inv[SIDE * SIDE];
        inv<T>(k_inv, k, SIDE);
        matarr_mul<T>(coord_3d, k_inv, coord_ext, SIDE, SIDE);
        arrval_mul<T>(coord_3d, coord_3d, d, SIDE);
        return coord_3d;
    }

    /**
     * @brief Vector wrapper of deproject.
     * @tparam T 
     * @param k 
     * @param coord 
     * @param d 
     * @return std::vector<T> 
     */
    template <typename T>
    std::vector<T> deproject(const std::vector<T> k, 
        const std::vector<T> coord, T d)
    {
        std::vector<T> ret(std::size_t{3});
        deproject<T>(ret.data(), k.data(), coord.data(), d);
        return ret;
    }
};

} // namespace EdgeLearning

#endif // MATH_HPP
