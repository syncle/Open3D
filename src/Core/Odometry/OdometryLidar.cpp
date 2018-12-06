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

#include "OdometryLidar.h"

#include <algorithm>
#include <iterator>
#include <numeric>      // std::iota
#include <vector>
#include <Eigen/Dense>

namespace open3d {

namespace {

// Eq (1)
void ComputeCurvature(const LidarScanLine& scan_line, std::vector<double> &curvature)
{
    int s = scan_line.points_.size();
    curvature.resize(s);
    // Eq (1)
    for (int j=0; j<s; j++) {
        double c_j = 0.0;
        auto point_j = scan_line.points_[j];
        for (int k=0; k<s; k++){
            if (j==k)
                continue;
            auto point_k = scan_line.points_[k];
            c_j += (point_j - point_k).norm();
        } 
        c_j /= (s*point_j.norm());
        curvature.push_back(c_j);
    }
}

// Fig. 4
void ComputeAngleAndDiscontinuity(const LidarScanLine& scan_line, 
        std::vector<double>& angle, std::vector<double>& discontinuity)
{
    int s = scan_line.points_.size();
    angle.resize(s);
    discontinuity.resize(s);
    for (int i=0; i<s; i++) {
        Eigen::Vector3d surface_grad;
        if (i==0){
            surface_grad = (scan_line.points_[1]-scan_line.points_[0]);
        }
        else if (i==s-1) {
            surface_grad = (scan_line.points_[s-1]-scan_line.points_[s-2]);
        }
        else {
            // is this the best way?
            surface_grad = (scan_line.points_[i+1]-scan_line.points_[i-1]);
        }
        Eigen::Vector3d scan_direction = scan_line.points_[i];
        scan_direction.normalize();               
        double norm_surface_grad = surface_grad.norm();
        surface_grad /= norm_surface_grad;               
        angle[i] = surface_grad.dot(scan_direction); // should do absolute?
        discontinuity[i] = norm_surface_grad;        
    }
}

// Section V-A
void DetectEdgePlanarFeature(const LidarScanLine& scan_line, const std::vector<double> &c, 
        std::vector<int>& edge, std::vector<int>& planar, const OdometryLidarOption &option)
{
    // seperate a scan into four identical subregions 
    // to evenly distribute the feature points
    int s = scan_line.points_.size();
    std::vector<double> angle, discontinuity;
    ComputeAngleAndDiscontinuity(scan_line, angle, discontinuity);
    assert(s % option.region_div_ != 0);
    int div = s / option.region_div_;        
    for (int j=0; j<option.region_div_; j++) {            
        auto first = c.begin() + div*j;
        auto last = first + div;
        std::vector<double> c_j(first, last);
        std::vector<int> index(div);
        std::iota(std::begin(index), std::end(index), div); // fill index with {0,1,2,...}
        sort(index.begin(), index.end(), [&c_j](size_t i, size_t j) {return c_j[i] < c_j[j];});
        // A point i can be selected as an edge or a planar point only if its c value 
        // is larger or smaller than a threshold.
        std::vector<int> edge_j;
        std::vector<int> planar_j;
        for (int k=0; k<div; k++) {
            if (angle[k] > option.angular_threshold_ && 
                    discontinuity[k] < option.depth_diff_threshold_) {
                if (c_j[k] > option.feature_selection_threshold_ && edge_j.size() < option.edge_feature_in_div_) {
                    edge_j.push_back(index[k]);                    
                    edge.push_back(index[k]+div*j);
                }
                if (c_j[k] < option.feature_selection_threshold_ && planar_j.size() < option.edge_feature_in_div_) {
                    planar_j.push_back(index[k]);
                    planar.push_back(index[k]+div*j);
                }
            }
        }
    }
}

// Eq (2)
inline double ComputeEdgeDistance(const LidarScan &source, const LidarScan &target, int line, int i, int j, int l) {
    auto X_k1_i = target.scan_lines_[line].points_[i];
    auto X_k_j = source.scan_lines_[line].points_[j];
    auto X_k_l = source.scan_lines_[line].points_[l];
    // add some verification to avoid dividing by zero
    auto temp0 = (X_k1_i-X_k_j).cross(X_k1_i-X_k_l);
    double temp1 = (X_k_j-X_k_l).norm();
    return temp0.norm()/temp1; // point to line distance
    // return 0;
}

// Eq (3)
inline double ComputePlanarPatchDistance(const LidarScan &source, const LidarScan &target, int line, int i, int j, int l, int m) {
    auto X_k1_i = target.scan_lines_[line].points_[i];
    auto X_k_j = source.scan_lines_[line].points_[j];
    auto X_k_l = source.scan_lines_[line].points_[l];
    auto X_k_m = source.scan_lines_[line].points_[m];
    // add some verification to avoid dividing by zero
    auto temp0 = ((X_k_j-X_k_l).cross(X_k_j-X_k_m));
    auto temp1 = (X_k1_i-X_k_j);
    return temp1.dot(temp0)/temp0.norm();  // point to plane distance. need to check the equation again
}

// Section V-A
void ComputeOdometry(const LidarScan &scan, const OdometryLidarOption &option)
{
    for (int i=0; i<scan.scan_lines_.size(); i++) {
        auto scan_line = scan.scan_lines_[i];
        int s = scan_line.points_.size();
        std::vector<double> curvature;
        ComputeCurvature(scan_line, curvature);
        std::vector<int> edge, planar;
        DetectEdgePlanarFeature(scan_line, curvature, edge, planar, option);
    }
}

}   // unnamed namespace

std::tuple<bool, Eigen::Matrix4d> ComputeOdometryLidar(
        const LidarScan &source, const LidarScan &target,
        const Eigen::Matrix4d &odo_init/* = Eigen::Matrix4d::Identity()*/,
        const OdometryLidarOption &option/* = OdometryLidarOption()*/) 
{
    return std::make_tuple(false, Eigen::Matrix4d::Identity());
}

}   // namespace open3d