/**
 * @file matrix_f32.cpp
 * @author lxlx (1729649497@qq.com)
 * @brief 
 * @version 0.1
 * @date 2026-11-08
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "matrix_f32.hpp"
#include "arm_math.h"

#include <cstring>

namespace matrix_utils {
namespace detail {

namespace {

static constexpr uint16_t kMaxInverseSize = 10U;

inline void initInstance(arm_matrix_instance_f32 &inst, uint16_t row, 
                         uint16_t col, float *data){
    arm_mat_init_f32(&inst, row, col, data);
}

} //namespace

void Transpose(uint16_t rows, uint16_t cols, const float *src, float *dst){
    arm_matrix_instance_f32 s;
    arm_matrix_instance_f32 d;
    initInstance(s, rows, cols, const_cast<float*>(src));
    initInstance(d, cols, rows, dst);
    (void)arm_mat_trans_f32(&s, &d);
}


bool Multiply(uint16_t rows_a, uint16_t cols_a, uint16_t cols_b, 
    const float *a, const float *b, float *dst){
    arm_matrix_instance_f32 ma;
    arm_matrix_instance_f32 mb;
    arm_matrix_instance_f32 md;

    initInstance(ma, rows_a, cols_a, const_cast<float*>(a));
    initInstance(mb, cols_a, cols_b, const_cast<float*>(b));
    initInstance(md, rows_a, cols_b, dst);

    return arm_mat_mult_f32(&ma, &mb, &md) == ARM_MATH_SUCCESS;
}

bool Inverse(uint16_t size, const float *src, float *dst){
    if(size > kMaxInverseSize || size == 0U){
        return false;
    }

    float work[kMaxInverseSize * kMaxInverseSize];

    std::memcpy(work, src, static_cast<size_t>(size) * 
            static_cast<size_t>(size) * sizeof(float));



    arm_matrix_instance_f32 s;
    arm_matrix_instance_f32 d;

    initInstance(s, size, size, work);
    initInstance(d, size, size, dst);

    return arm_mat_inverse_f32(&s, &d) == ARM_MATH_SUCCESS;
}

} //namespace detail
} //namespace matrix_utils



