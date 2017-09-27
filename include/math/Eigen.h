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

using Array2f = Eigen::Array2f;
using Array3f = Eigen::Array3f;

using Vector2f = Eigen::Vector2f;
using Vector3f = Eigen::Vector3f;
using Vector4f = Eigen::Vector4f;
using Vector5f = Eigen::Matrix<float, 5, 1>;
using Vector6f = Eigen::Matrix<float, 6, 1>;
using VectorXf = Eigen::VectorXf;

using RowVector2f = Eigen::RowVector2f;
using RowVector3f = Eigen::RowVector3f;
using RowVector4f = Eigen::RowVector4f;
using RowVectorXf = Eigen::RowVectorXf;

using Matrix2x2f = Eigen::Matrix2f;
using Matrix2x3f = Eigen::Matrix<float, 2, 3>;
using Matrix2x4f = Eigen::Matrix<float, 2, 4>;
using Matrix3x2f = Eigen::Matrix<float, 3, 2>;
using Matrix3x3f = Eigen::Matrix3f;
using Matrix4x2f = Eigen::Matrix<float, 4, 2>;
using Matrix4x3f = Eigen::Matrix<float, 4, 3>;
using Matrix4x4f = Eigen::Matrix4f;
using MatrixXf = Eigen::MatrixXf;

using Quaternionf = Eigen::Quaternionf;
using AngleAxisf = Eigen::AngleAxisf;