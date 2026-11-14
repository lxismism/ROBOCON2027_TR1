/**
 * @file matrix_f32.hpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-11-08
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cstdint>
#include <array>
#include <cstddef>



namespace matrix_utils{

template <uint16_t Rows, uint16_t Cols>
class Matrix{

public:

    static constexpr uint16_t kRows = Rows;
    static constexpr uint16_t kCols = Cols;
    static constexpr uint16_t kSize = kRows * kCols;
    
    std::array<float, kSize> data_{};
    
    constexpr float &operator()(uint16_t row, uint16_t col){
        return data_[row * kCols + col];
    }

    constexpr const float &operator()(uint16_t row, uint16_t col) const {
        return data_[row * kCols + col];
    }
    
};


namespace detail{
    void Transpose(uint16_t rows, uint16_t cols, const float *src, float *dst);
    bool Multiply(uint16_t rows_a, uint16_t cols_a, uint16_t cols_b, const float *a, const float *b, float *dst);
    bool Inverse(uint16_t size, const float *src, float *dst);
}   //namespace detail

template <uint16_t Size> using Vector = Matrix<Size, 1>;    //只有一列的矩阵定义为向量

/**
 * @brief 矩阵二元加法
 * 
 * @tparam Rows 
 * @tparam Cols 
 * @param a 
 * @param b 
 * @return Matrix<Rows, Cols> 
 */
template<uint16_t Rows, uint16_t Cols>
inline Matrix<Rows, Cols> operator+(const Matrix<Rows, Cols> &a, const Matrix<Rows, Cols> &b){
    Matrix<Rows, Cols> out{};
    for (size_t i = 0; i < out.data_.size(); ++i) {
        out.data_[i] = a.data_[i] + b.data_[i];
    }
    return out;
}

/**
 * @brief 矩阵二元减法
 * 
 * @tparam Rows 
 * @tparam Cols 
 * @param a 
 * @param b 
 * @return Matrix<Rows, Cols> 
 */
template<uint16_t Rows, uint16_t Cols>
inline Matrix<Rows, Cols> operator-(const Matrix<Rows, Cols> &a, const Matrix<Rows, Cols> &b){
    Matrix<Rows, Cols> out{};
    for(size_t i = 0; i < out.data_.size(); ++i){
        out.data_[i] = a.data_[i] - b.data_[i];
    }
    return out;
}

/**
 * @brief 矩阵乘法
 * 
 * @tparam Rows_a 
 * @tparam Cols_a 
 * @tparam Cols_b 
 * @param a 
 * @param b 
 * @return Matrix<Rows_a, Cols_b> 
 */
template<uint16_t Rows_a, uint16_t Cols_a, uint16_t Cols_b>
inline Matrix<Rows_a, Cols_b> operator*(const Matrix<Rows_a, Cols_a> &a, const Matrix<Cols_a, Cols_b> &b){
    Matrix<Rows_a, Cols_b> out{};
    (void)detail::Multiply(Rows_a, Cols_a, Cols_b, a.data_.data(), b.data_.data(), out.data_.data());
    return out;
}

/**
 * @brief 下面两个函数都是矩阵的二元数乘，为了满足交换律所以写了两个
 * 
 * @tparam Rows 
 * @tparam Cols 
 * @param k 
 * @param dst 
 * @return Matrix<Rows, Cols> 
 */
template <uint16_t Rows, uint16_t Cols>
inline Matrix<Rows, Cols> operator*(float k,const Matrix<Rows, Cols> &dst){
    Matrix<Rows, Cols> out{};
    for(size_t i = 0; i < out.data_.size(); ++i){
        out.data_[i] = k * dst.data_[i];
    }
    return out;
}

template <uint16_t Rows, uint16_t Cols>
inline Matrix<Rows, Cols> operator*(const Matrix<Rows, Cols> &dst, float k){
    Matrix<Rows, Cols> out{};
    for(size_t i = 0; i < out.data_.size(); ++i){
        out.data_[i] = dst.data_[i] * k;
    }
    return out;
}

/**
 * @brief 矩阵一元取负
 * 
 * @tparam Rows 
 * @tparam Cols 
 * @param dst 
 * @return Matrix<Rows, Cols> 
 */
template <uint16_t Rows, uint16_t Cols>
inline Matrix<Rows, Cols> operator-(const Matrix<Rows, Cols> &dst){
    Matrix<Rows, Cols> out{};
    for(size_t i = 0; i < out.data_.size(); ++i){
        out.data_[i] = -dst.data_[i];
    }
    return out;
}

/**
 * @brief 矩阵转置
 * 
 * @tparam Rows 
 * @tparam Cols 
 * @param dst 
 * @return Matrix<Cols, Rows> 
 */
template <uint16_t Rows, uint16_t Cols>
inline Matrix<Cols, Rows> Trans(const Matrix<Rows, Cols> &dst){
    Matrix<Cols, Rows> out{};

    detail::Transpose(Rows, Cols, dst.data_.data(), out.data_.data());

    return out;
}

/**
 * @brief 矩阵求逆(便携版)
 * 
 * @tparam Size 
 * @param dst 
 * @return Matrix<Size, Size> 
 */
template <uint16_t Size>
inline Matrix<Size, Size> Inv(const Matrix<Size, Size> &dst){
    Matrix<Size, Size> out{};
    (void)detail::Inverse(Size, dst.data_.data(), out.data_.data());
    return out;
}

/**
 * @brief 矩阵求逆(安全版，奇异点返回false)
 * 
 * @tparam Size 
 * @param src 
 * @param dst 
 * @return true 
 * @return false 
 */
template<uint16_t Size>
inline bool Inv(const Matrix<Size, Size> &src, Matrix<Size, Size> &dst){
    return detail::Inverse(Size, src.data_.data(), dst.data_.data());
}


}