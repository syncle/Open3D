// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "OdometryLidarJacobian.h"

#include <vector>
#include <tuple>

namespace open3d {

void ComputeJacobianAndResidualForEdgeFeatures(int row, Eigen::Vector6d &J_r,
                                double &r, const LidarScan &source,
                                const LidarScan &target,
                                const std::vector<LiderScanPointCorrespondence> &edge_corr) {
    auto source_point = edge_corr[row].source_point_;
    auto target_point_0 = edge_corr[row].target_points_[0];
    auto target_point_1 = edge_corr[row].target_points_[1];

    int line_i = source_point.scan_line_id_;
    int i = source_point.vertex_id_;
    int line_j = target_point_0.scan_line_id_;
    int j = target_point_0.vertex_id_;
    int line_l = target_point_1.scan_line_id_;
    int l = target_point_1.vertex_id_;

    auto X_i = target.scan_lines_[line_i].points_[i];
    auto X_j = source.scan_lines_[line_j].points_[j];
    auto X_l = source.scan_lines_[line_l].points_[l];
    
    // add some verification to avoid dividing by zero
    auto temp0 = (X_i - X_j).cross(X_i - X_l);
    double temp1 = (X_j - X_l).norm();

    // edge term
    J_r = Eigen::Vector6d::Zero();
    Eigen::Vector3d f = temp0 / temp0.norm();
    Eigen::Matrix3d dx_dxi = Eigen::Matrix3d::Zero(); // more fancier 3x3 matrix?
    dx_dxi(0, 1) = -X_l(2) - X_j(2);
    dx_dxi(0, 2) = -X_j(1) - X_l(1);
    dx_dxi(1, 0) = X_l(2) - X_j(2);
    dx_dxi(1, 2) = X_j(0) - X_l(0);
    dx_dxi(2, 0) = -X_j(1) - X_l(1);
    dx_dxi(2, 1) = -X_j(0) - X_l(0);
    Eigen::MatrixXd dxi_dT(3,6);
    dxi_dT.setZero();
    dxi_dT(0, 1) = X_i(2);
    dxi_dT(0, 2) = -X_i(1);
    dxi_dT(0, 3) = 1.0;
    dxi_dT(1, 0) = -X_i(2);
    dxi_dT(1, 2) = X_i(0);
    dxi_dT(1, 4) = 1.0;
    dxi_dT(2, 0) = X_i(1);
    dxi_dT(2, 1) = -X_i(0);
    dxi_dT(2, 5) = 1.0;  // this matrix can be reused. How this is defined in
                         // other examples?
    J_r = 1 / temp1 * f * dx_dxi * dxi_dT;
    r = temp0.norm() / temp1;  // point to line distance, relative contribution based on rotation angle.
}

void ComputeJacobianAndResidualForPlanarFeatures(
    int row, Eigen::Vector6d &J_r, double &r,
    const LidarScan &source, const LidarScan &target,
    const std::vector<LiderScanPointCorrespondence> &edge_corr) 
{
    auto source_point = edge_corr[row].source_point_;
    auto target_point_0 = edge_corr[row].target_points_[0];
    auto target_point_1 = edge_corr[row].target_points_[1];
    auto target_point_2 = edge_corr[row].target_points_[2];

    int line_i = source_point.scan_line_id_;
    int i = source_point.vertex_id_;
    int line_j = target_point_0.scan_line_id_;
    int j = target_point_0.vertex_id_;
    int line_l = target_point_1.scan_line_id_;
    int l = target_point_1.vertex_id_;
    int line_m = target_point_2.scan_line_id_;
    int m = target_point_2.vertex_id_;

    auto X_i = target.scan_lines_[line_i].points_[i]; // source <-> target ?
    auto X_j = source.scan_lines_[line_j].points_[j];
    auto X_l = source.scan_lines_[line_l].points_[l];
    auto X_m = source.scan_lines_[line_m].points_[m];

    auto temp0 = ((X_j - X_l).cross(X_j - X_m));
    temp0.normalize();
    auto temp1 = (X_i - X_j);
    r = temp1.dot(temp0);

    // planar term
    J_r = Eigen::Vector6d::Zero();
    Eigen::MatrixXd dxi_dT(3, 6);
    dxi_dT.setZero();
    dxi_dT(0, 1) = X_i(2);
    dxi_dT(0, 2) = -X_i(1);
    dxi_dT(0, 3) = 1.0;
    dxi_dT(1, 0) = -X_i(2);
    dxi_dT(1, 2) = X_i(0);
    dxi_dT(1, 4) = 1.0;
    dxi_dT(2, 0) = X_i(1);
    dxi_dT(2, 1) = -X_i(0);
    dxi_dT(2, 5) = 1.0;  // this matrix can be reused. How this is defined in
                         // other examples?
    J_r = temp0 * dxi_dT;
    // r = temp0.norm() / temp1;  // point to point distance, relative contribution
                               // based on rotation angle.
}

}  // namespace open3d