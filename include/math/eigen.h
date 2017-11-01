/**
 * Copyright 2016 Arseniy Ivin <arssivka@yandex.ru>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  @autor arssivka
 *  @date 8/14/17
 */

#pragma once


#include <Eigen/Dense>

using array_2f_t = Eigen::Array2f;
using array_3f_t = Eigen::Array3f;

using vector_2f_t = Eigen::Vector2f;
using vector_3f_t = Eigen::Vector3f;
using vector_4f_t = Eigen::Vector4f;
using vector_5f_t = Eigen::Matrix<float, 5, 1>;
using vector_6f_t = Eigen::Matrix<float, 6, 1>;
using vector_Xf_t = Eigen::VectorXf;

using row_vector_2f_t = Eigen::RowVector2f;
using row_vector_3f_t = Eigen::RowVector3f;
using row_vector_4f_t = Eigen::RowVector4f;
using row_vector_xf_t = Eigen::RowVectorXf;

using matrix_2x2f_t = Eigen::Matrix2f;
using matrix_2x3f_t = Eigen::Matrix<float, 2, 3>;
using matrix_2x4f_t = Eigen::Matrix<float, 2, 4>;
using matrix_3x2f_t = Eigen::Matrix<float, 3, 2>;
using matrix_3x3f_t = Eigen::Matrix3f;
using matrix_4x2f_t = Eigen::Matrix<float, 4, 2>;
using matrix_4x3f_t = Eigen::Matrix<float, 4, 3>;
using matrix_4x4f_t = Eigen::Matrix4f;
using matrix_xf_t = Eigen::MatrixXf;

using quaternion_f_t = Eigen::Quaternionf;
using angle_axis_f_t = Eigen::AngleAxisf;