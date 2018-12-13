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
// #include <Core/Odometry/OdometryLidarOption.h>
#include <Core/Odometry/OdometryLidarJacobian.h>
#include <Core/Geometry/KDTreeFlann.h>
#include <Core/Geometry/PointCloud.h>
#include <Core/Utility/Console.h>

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
        curvature.push_back(c_j); // no need to compute curvature using every points
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
        discontinuity[i] = norm_surface_grad;        // we should only pick OCCLUDED points - should consider minus?
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

std::shared_ptr<KDTreeFlann> MakeFeatureKDTree(
        const LidarScanLine &scan_line, const std::vector<int> ind)
{
    PointCloud temp_pcd;
    for (auto point: scan_line.points_) {
        temp_pcd.points_.push_back(point);
    }
    std::shared_ptr<KDTreeFlann> kd_tree;
    kd_tree->SetGeometry(temp_pcd);
    return kd_tree;
}

// Section V-A
typedef std::vector<std::shared_ptr<KDTreeFlann>> FeatureTree;
typedef std::vector<std::vector<int>> FeatureIndex;
typedef std::tuple<FeatureTree, FeatureIndex> LidarFeature;
std::tuple<LidarFeature, LidarFeature> Preprocessing(const LidarScan &scan, const OdometryLidarOption &option)
{
    // line-by-line processing
    FeatureTree edge_trees, planar_trees;
    FeatureIndex edge_indices, planar_indices;
    for (int i=0; i<scan.scan_lines_.size(); i++) {
        auto scan_line = scan.scan_lines_[i];
        int s = scan_line.points_.size();
        std::vector<double> curvature;
        ComputeCurvature(scan_line, curvature);
        std::vector<int> edge, planar;
        DetectEdgePlanarFeature(scan_line, curvature, edge, planar, option);
        edge_indices.push_back(edge);
        planar_indices.push_back(planar);
        edge_trees.push_back(MakeFeatureKDTree(scan_line, edge));
        planar_trees.push_back(MakeFeatureKDTree(scan_line, planar));
    }
    auto edge_feature = std::make_tuple(edge_trees, edge_indices);
    auto planar_feature = std::make_tuple(planar_trees, planar_indices);
    return std::make_tuple(edge_feature, planar_feature);
}

enum MatchingMode {
    EDGE = 0,
    PLANAR = 1
};

typedef std::vector<LiderScanPointCorrespondence> Correspondences;
Correspondences GetMatchingPoints(
        const FeatureTree &source_tree, const FeatureIndex &source_index, 
        const FeatureTree &target_tree, const FeatureIndex &target_index, 
        const LidarScan &source, MatchingMode mode)
{
    int n_scan_lines = source_tree.size();  // source_tree is not really necessary. Can we do this better?

    std::vector<LiderScanPointCorrespondence> output;
    for (auto line=0; line<n_scan_lines; line++) {
        auto si_line = source_index[line];
        for (auto i=0; i<si_line.size(); i++) {

            LiderScanPointCorrespondence corr;
            corr.source_point_.scan_line_id_ = line;
            corr.source_point_.vertex_id_ = i;

            // can we do this better using auto tree_scan_line: source_tree_planar?
            // auto sf_i = source_tree_edge[i];
            std::vector<int> indices;
            std::vector<double> distance2;
            auto query_i = source.scan_lines_[line].points_[si_line[i]];
            double min_dist = 10000.0;
            
            // find the point that is nearest to the point i
            int min_scan_line_j, min_j, min_scan_line_m, min_m;            
            for (auto line2=0; line2<n_scan_lines; line2++) {
                // auto ti_line = target_index_edge[line]; // in global index domain?
                if (line == line2) // not searching the same line
                    continue;
                auto tf_line = target_tree[line2];
                tf_line->SearchKNN(query_i, 1, indices, distance2);
                if (distance2[0] < min_dist) {
                    min_scan_line_j = line2;
                    min_j = indices[0];
                    min_dist = distance2[0];
                }
            }
            corr.target_points_.resize(2);
            corr.target_points_[0].scan_line_id_ = min_scan_line_j;
            corr.target_points_[0].vertex_id_ = min_j;

            // find the point l that is one of the two consecutive lines of j
            min_dist = 10000.0;
            int min_scan_line_l, min_l;
            for (auto line3=min_scan_line_j-1; line3<=min_scan_line_j+1; line3++) {
                if (line3 == min_scan_line_j || line3 < 0 || line3 > n_scan_lines-1)
                    continue;
                auto tf_line = target_tree[line3];
                tf_line->SearchKNN(query_i, 1, indices, distance2);
                if (distance2[0] < min_dist) {
                    min_scan_line_l = line3;
                    min_l = indices[0];
                    min_dist = distance2[0];
                }
            }
            corr.target_points_[1].scan_line_id_ = min_scan_line_l;
            corr.target_points_[1].vertex_id_ = min_l;

            if (mode == PLANAR) {  // pick additional point to make a plane
                auto tf_line = target_tree[min_scan_line_j];
                tf_line->SearchKNN(query_i, 1, indices, distance2);
                min_scan_line_m = min_scan_line_j;
                min_m = indices[1];
                LiderScanPoint temp;
                temp.scan_line_id_ = min_scan_line_m;
                temp.vertex_id_ = min_m;
                corr.target_points_.push_back(temp);
            }

            output.push_back(corr);
        }   
    }
    return std::move(output);
}

std::tuple<Correspondences, Correspondences> Matching(
        const LidarFeature &source_feature_edge, const LidarFeature &source_feature_planar, 
        const LidarFeature &target_feature_edge, const LidarFeature &target_feature_planar, 
        const LidarScan &source, const LidarScan &target)
{
    auto edge_corr = GetMatchingPoints(
            std::get<0>(source_feature_edge), std::get<1>(source_feature_edge), 
            std::get<0>(target_feature_edge), std::get<1>(target_feature_edge), source, EDGE);
    auto planar_corr = GetMatchingPoints(
            std::get<0>(source_feature_planar), std::get<1>(source_feature_planar),
            std::get<0>(target_feature_planar), std::get<1>(target_feature_planar),
            source, PLANAR);
    return std::make_tuple(edge_corr, planar_corr);
}

std::tuple<bool, Eigen::Matrix4d> Execution(const LidarScan &source,
                                            const LidarScan &target,
                                            const OdometryLidarOption &option) {
    // need to do some warping for source point cloud.
    auto source_feature = Preprocessing(source, option); // devide preprocessing components - not all components are being used.
    auto target_feature = Preprocessing(target, option);
    Correspondences edge_corr, planar_corr;
    std::tie(edge_corr, planar_corr) = Matching(
            std::get<0>(source_feature), std::get<1>(source_feature),
            std::get<0>(target_feature), std::get<1>(target_feature), 
            source, target);

    // retrieve 6D motion
    auto f_edge = [&](int i, Eigen::Vector6d &J_r, double &r) {
        ComputeJacobianAndResidualForEdgeFeatures(i, J_r, r, source, target, edge_corr);
    };
    auto f_planar = [&](int i, Eigen::Vector6d &J_r, double &r) {
        ComputeJacobianAndResidualForPlanarFeatures(i, J_r, r, source, target, planar_corr);
    };
    // PrintDebug("Iter : %d, Level : %d, ", iter, level);
    Eigen::Matrix6d JTJ, JTJ_edge, JTJ_planar;
    Eigen::Vector6d JTr, JTr_edge, JTr_planar;
    std::tie(JTJ_edge, JTr_edge) = ComputeJTJandJTr<Eigen::Matrix6d, Eigen::Vector6d>(
            f_edge, edge_corr.size());
    std::tie(JTJ_planar, JTr_planar) = ComputeJTJandJTr<Eigen::Matrix6d, Eigen::Vector6d>(
            f_planar, planar_corr.size());
    JTJ = JTJ_edge + JTJ_planar;
    JTr = JTr_edge + JTr_planar;

    bool is_success;
    Eigen::Matrix4d extrinsic;
    std::tie(is_success, extrinsic) =
        SolveJacobianSystemAndObtainExtrinsicMatrix(JTJ, JTr);
    if (!is_success) {
        PrintWarning("[ComputeOdometryLidar] no solution!\n");
        return std::make_tuple(false, Eigen::Matrix4d::Identity());
    } else {
        return std::make_tuple(true, extrinsic);
    }
}

}   // unnamed namespace

std::tuple<bool, Eigen::Matrix4d> ComputeOdometryLidar(
        const LidarScan &source, const LidarScan &target,
        const Eigen::Matrix4d &odo_init/* = Eigen::Matrix4d::Identity()*/,
        const OdometryLidarOption &option/* = OdometryLidarOption()*/) 
{
    return Execution(source, target, option);
}

}   // namespace open3d