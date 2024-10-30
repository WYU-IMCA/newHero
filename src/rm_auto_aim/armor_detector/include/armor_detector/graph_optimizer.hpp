// Created by Labor 2023.8.25
// Maintained by Labor, Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARMOR_DETECTOR_GRAPH_OPTIMIZER_HPP_
#define ARMOR_DETECTOR_GRAPH_OPTIMIZER_HPP_

// std
#include <array>
// g2o
#include <g2o/core/auto_differentiation.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/sparse_optimizer.h>
// 3rd party
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
// project
#include "armor_detector/types.hpp"

namespace imca::auto_aim {
// Vertex of graph optimization algorithm for the yaw angle
class VertexYaw : public g2o::BaseVertex<1, Eigen::Vector<double, 1>> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  VertexYaw() = default;
  virtual void setToOriginImpl() override { _estimate.setZero(); }
  virtual void oplusImpl(const double *update) override;

  virtual bool read(std::istream &in) override { return true; }
  virtual bool write(std::ostream &out) const override { return true; }
};

// Edge of graph optimization algorithm for reporjection error calculation using yaw angle and observation
class EdgeProjection : public g2o::BaseUnaryEdge<Armor::N_LANDMARKS_2,
                                                 Eigen::Vector<double, Armor::N_LANDMARKS_2>,
                                                 VertexYaw> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using InfoMatrixType = Eigen::Matrix<double, Armor::N_LANDMARKS_2, Armor::N_LANDMARKS_2>;

  EdgeProjection(const Sophus::SO3d &camera2imu,
                 const Eigen::Vector3d &t,
                 const CameraInternalK &k,
                 const Eigen::Vector2d &size,
                 const double pitch);
  virtual void computeError() override;

  virtual bool read(std::istream &in) override { return true; }
  virtual bool write(std::ostream &out) const override { return true; }

private:
  Sophus::SO3d camera2imu_;
  Eigen::Vector3d t_;
  CameraInternalK K_;
  std::vector<Eigen::Vector3d> object_points_; 
  double pitch_;
};

}  // namespace imca::auto_aim
#endif  // ARMOR_DETECTOR_GRAPH_OPTIMIZER_HPP_
