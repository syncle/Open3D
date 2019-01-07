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
#include <Core/Utility/Eigen.h>

// for debugging
#include <iostream>
#include <fstream>

namespace open3d {

namespace {

std::vector<int> CreateIndexVector(int n)
{
    std::vector<int> index(n);
    // fill index with {0,1,2,...}
    std::iota(std::begin(index), std::end(index), 0);
    return std::move(index);
}

// Eq (1)
void ComputeCurvatureLine(const LidarScanLine& scan_line, 
        std::vector<double> &curvature,
        const OdometryLidarOption &option)
{
    int n_points = scan_line.points_.size();
    curvature.resize(n_points);
    auto index = CreateIndexVector(n_points);
    for (int j = 0; j < n_points; j++) {
        if (!scan_line.is_valid_[j]) {
            curvature[j] = 0.0;
            continue;
        }
        int start = std::max(0, j - option.curvature_window_half_size_);
        int end = std::min(n_points - 1, j + option.curvature_window_half_size_);
        std::vector<int> index_j(&index[start], &index[end]);
        auto pt_j = scan_line.points_[j];
        int n_valid = 0;
        double sum = 0.0;
        for (int i : index_j) {
            if (scan_line.is_valid_[i]) {
                sum += (scan_line.points_[i] - pt_j).norm();
                n_valid++;
            }
        }
        double norm = pt_j.norm();
        if (n_valid != 0 && norm != 0.0)
            curvature[j] = sum / (n_valid * norm);
        else
            curvature[j] = 0.0;
    }
}

// Fig. 4
void ComputeAngleAndDiscontinuity(const LidarScanLine& scan_line, 
        std::vector<double>& angle, std::vector<double>& discontinuity)
{
    int n_points = scan_line.points_.size();
    angle.resize(n_points);
    discontinuity.resize(n_points);
    for (int i = 0; i < n_points; i++) {
        Eigen::Vector3d surface_grad;
        int i_m_1 = std::max(0, i - 1);
        int i_p_1 = std::min(n_points - 1, i + 1);
        if (scan_line.is_valid_[i] && 
            scan_line.is_valid_[i_m_1] &&
            scan_line.is_valid_[i_p_1]) 
        {
            surface_grad = scan_line.points_[i_p_1] - scan_line.points_[i_m_1];
            Eigen::Vector3d scan_direction = scan_line.points_[i];
            scan_direction.normalize();
            double norm_surface_grad = surface_grad.norm();
            if (norm_surface_grad > 0.0) 
                surface_grad /= norm_surface_grad;
            angle[i] = std::abs(surface_grad.dot(scan_direction));
            discontinuity[i] = norm_surface_grad;  // we should only pick OCCLUDED points -
        } else {
            angle[i] = 1.0;
            discontinuity[i] = 9999.0;  // smarter?
        }
    }
}

void save_double_vector_for_debugging(const std::vector<std::vector<double>>& input, const char* file_name) {
    PrintError("save_double_vector_for_debugging\n");
    std::ofstream myfile;
    myfile.open(file_name);
    int line_cnt = 0;
    for (auto line : input) {
        for (auto val : line) {
            myfile << std::to_string(val) << "\n";
        }
    }
    myfile.close();
}

// Section V-A
void DetectEdgePlanarFeatureLine(const LidarScanLine& scan_line, const std::vector<double> &curvature, 
        std::vector<int>& edge, std::vector<int>& planar, const OdometryLidarOption &option,
        std::vector<double>& angular, std::vector<double>& depth)
{
    // seperate a scan into four identical subregions 
    // to evenly distribute the feature points
    int s = scan_line.points_.size();
    std::vector<double> angle, discontinuity;
    ComputeAngleAndDiscontinuity(scan_line, angle, discontinuity);
    assert(s % option.region_div_ != 0);
    int n_division = s / option.region_div_; // need to handle non-integer case

    for (int d=0; d<option.region_div_; d++) {
        auto first = curvature.begin() + (n_division * d);
        auto last = first + n_division + 1;
        std::vector<double> curvature_d(first, last);
        std::vector<double> angle_d(first, last);
        std::vector<double> discontinuity_d(first, last);

        // sort curvature values
        auto index = CreateIndexVector(n_division);
        sort(index.begin(), index.end(), [&curvature_d](size_t i, size_t j) {
            return curvature_d[i] < curvature_d[j];
        });

        // A point i can be selected as an edge or a planar point only if its c
        // value is larger or smaller than a threshold.
        // planar feature detection
        int cnt_div_planar_feature = 0;
        for (int i = 0; i < n_division; i++) { // small to large curvature value
            int i_sorted = index[i];
            int i_global = i_sorted + n_division * d;
            if (!scan_line.is_valid_[i_global])
                continue;
            if (angle_d[i_sorted] < option.angular_threshold_ && 
                    discontinuity_d[i_sorted] < option.depth_diff_threshold_) {
                if (curvature_d[i_sorted] < option.feature_selection_threshold_ &&
                    cnt_div_planar_feature < option.planar_feature_in_div_) {
                    planar.push_back(i_global);
                    cnt_div_planar_feature++;
                }
            }
        }
        // edge feature detection
        int cnt_div_edge_feature = 0;
        for (int i = n_division - 1; i >= 0; i--) { // large to small curvature value
            int i_sorted = index[i];
            int i_global = i_sorted + n_division * d;
            if (!scan_line.is_valid_[i_global])
                continue;
            if (angle_d[i_sorted] < option.angular_threshold_ && 
                    discontinuity_d[i_sorted] < option.depth_diff_threshold_) {
                if (curvature_d[i_sorted] >= option.feature_selection_threshold_ &&
                    cnt_div_edge_feature < option.edge_feature_in_div_) {
                    edge.push_back(i_global);
                    cnt_div_edge_feature++;
                }
            }
        }
    }
    angular = angle;        // for debugging
    depth = discontinuity;  // for debugging
}

// Section V-A
void DetectEdgePlanarPointLine(const LidarScanLine& scan_line, const std::vector<double> &curvature, 
        std::vector<int>& edge, std::vector<int>& planar, const OdometryLidarOption &option,
        std::vector<double>& angular, std::vector<double>& depth)
{
    // seperate a scan into four identical subregions 
    // to evenly distribute the feature points
    std::vector<double> angle, discontinuity;
    ComputeAngleAndDiscontinuity(scan_line, angle, discontinuity);

    for (int i = 0; i < scan_line.points_.size(); i++) {
        if (!scan_line.is_valid_[i]) 
            continue;
        if (curvature[i] != 0.0 && angle[i] < option.angular_threshold_ &&
                discontinuity[i] < option.depth_diff_threshold_) {
            if (curvature[i] < option.feature_selection_threshold_) {
                planar.push_back(i);
            }
            if (curvature[i] >= option.feature_selection_threshold_) {
                edge.push_back(i);
            }
        }
    }
}

std::shared_ptr<KDTreeFlann> MakeKDTree(
        const LidarScanLine &scan_line, const std::vector<int> ind)
{
    PointCloud temp_pcd;
    for (auto i : ind) {
        if (!scan_line.is_valid_[i]) 
            continue;
        temp_pcd.points_.push_back(scan_line.points_[i]);
    }
    if (ind.size() == 0) {
        return std::make_shared<KDTreeFlann>();
    } else {
        auto kd_tree = std::make_shared<KDTreeFlann>();
        kd_tree->SetGeometry(temp_pcd);
        return kd_tree;
    }
}

// Section V-A
typedef std::vector<std::shared_ptr<KDTreeFlann>> FeatureTree;
typedef std::vector<std::vector<int>> FeatureIndex;
typedef std::vector<std::vector<double>> FeatureValue;
// typedef std::tuple<FeatureTree, FeatureIndex> LidarFeature;

std::tuple<FeatureIndex, FeatureIndex> Preprocessing(
        const LidarScan &scan, bool is_feature_extraction,
        const OdometryLidarOption &option) {
    // line-by-line processing
    FeatureIndex edge_indices, planar_indices;

    std::vector<std::vector<double>> curvature;       // for debugging
    std::vector<std::vector<double>> angular;         // for debugging
    std::vector<std::vector<double>> depth;           // for debugging

    for (int i=0; i<scan.scan_lines_.size(); i++) {
        auto scan_line = scan.scan_lines_[i];
        std::vector<double> curvature_i;
        ComputeCurvatureLine(scan_line, curvature_i, option);     
        curvature.push_back(curvature_i);               // for debugging
        
        std::vector<int> edge, planar;
        std::vector<double> angular_i, depth_i;         // for debugging
        if (is_feature_extraction)
            DetectEdgePlanarFeatureLine(scan_line, curvature_i, edge, planar,
                    option, angular_i, depth_i);
        else
            DetectEdgePlanarPointLine(scan_line, curvature_i, edge, planar,
                    option, angular_i, depth_i);
        angular.push_back(angular_i);
        depth.push_back(depth_i);

        edge_indices.push_back(edge);
        planar_indices.push_back(planar);
    }
    
    // save_double_vector_for_debugging(curvature, "curvature.txt"); // for debugging
    // save_double_vector_for_debugging(angular, "angular.txt");     // for debugging
    // save_double_vector_for_debugging(depth, "depth.txt");         // for debugging

    return std::make_tuple(edge_indices, planar_indices);
}

std::tuple<FeatureTree, FeatureTree> MakePointTrees(const LidarScan &scan, 
        const FeatureIndex &edge, const FeatureIndex &planar)
{
    FeatureTree edge_trees, planar_trees;
    for (int i = 0; i < scan.scan_lines_.size(); i++) {
        PrintError("line : %d\n", i);
        auto scan_line = scan.scan_lines_[i];
        edge_trees.push_back(MakeKDTree(scan_line, edge[i]));
        planar_trees.push_back(MakeKDTree(scan_line, planar[i]));
    }
    return std::make_tuple(edge_trees, planar_trees);
}

void save_feature_for_debugging(const FeatureIndex &index,
                                const char *file_name) {
    PrintError("save_feature_for_debugging\n");
    std::ofstream myfile;
    myfile.open(file_name);
    int line_cnt = 0;
    for(auto line : index) {
        for (auto id : line) {
            myfile << std::to_string(line_cnt * 870 + id) << "\n";
        }
        line_cnt++;
    }
    myfile.close();
}

enum MatchingMode {
    EDGE = 0,
    PLANAR = 1
};

typedef std::vector<LiderScanPointCorrespondence> Correspondences;
Correspondences GetMatchingPoints(
        const FeatureIndex &source_index,
        const FeatureIndex &target_index,
        const LidarScan &source,
        const FeatureTree &target_tree, MatchingMode mode)
{
    int n_scan_lines = target_tree.size();

    std::vector<LiderScanPointCorrespondence> output;
    for (auto line=0; line<n_scan_lines; line++) {
        auto si_line = source_index[line];
        for (auto i: si_line) {
            LiderScanPointCorrespondence corr;
            corr.source_point_.scan_line_id_ = line;
            corr.source_point_.vertex_id_ = i;

            // can we do this better using auto tree_scan_line: source_tree_planar?
            // auto sf_i = source_tree_edge[i];
            auto query_i = source.scan_lines_[line].points_[i];
            double min_dist = 10000.0;
            
            // find the point that is nearest to the point i
            int min_scan_line_j = 0, min_j = 0;
            for (auto line2=0; line2<n_scan_lines; line2++) {
                // auto ti_line = target_index_edge[line]; // in global index domain?
                // if (line == line2) // not searching the same line - this is target line
                //     continue;
                auto target_tree_line = target_tree[line2];
                std::vector<int> indices(1);
                std::vector<double> distance2(1);
                int k = target_tree_line->SearchKNN(
                        query_i, 1, indices, distance2);
                if (k < 1)
                    continue;
                if (distance2[0] < min_dist) {
                    min_scan_line_j = line2;
                    min_j = target_index[line2][indices[0]];
                    min_dist = distance2[0];
                }
            }
            corr.target_points_.resize(2);
            corr.target_points_[0].scan_line_id_ = min_scan_line_j;
            corr.target_points_[0].vertex_id_ = min_j;

            // find the point l that is one of the two consecutive lines of j
            min_dist = 10000.0;
            int min_scan_line_l = 0, min_l = 0; // issue what if there is no l? 
            for (auto line3=min_scan_line_j-1; line3<=min_scan_line_j+1; line3++) {
                if (line3 == min_scan_line_j || line3 < 0 || line3 > n_scan_lines-1)
                    continue;
                auto target_tree_line = target_tree[line3];
                std::vector<int> indices(1);
                std::vector<double> distance2(1);
                int k = target_tree_line->SearchKNN(
                        query_i, 1, indices, distance2);
                if (k < 1)
                    continue;
                if (distance2[0] < min_dist) {
                    min_scan_line_l = line3;
                    min_l = target_index[line3][indices[0]];
                    min_dist = distance2[0];
                }
            }
            corr.target_points_[1].scan_line_id_ = min_scan_line_l;
            corr.target_points_[1].vertex_id_ = min_l;

            int min_scan_line_m = 0, min_m = 0;
            if (mode == PLANAR) {  // pick additional point to make a plane
                min_scan_line_m = min_scan_line_j;
                auto tf_line = target_tree[min_scan_line_m];
                std::vector<int> indices(2);
                std::vector<double> distance2(2);
                int k = tf_line->SearchKNN(query_i, 2, indices, distance2);
                if (k < 2)
                    continue;
                min_m = target_index[min_scan_line_m][indices[1]];
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

void save_corr_for_debugging(const Correspondences &corr,
                             const char *file_name, int i_th_feature) {
    PrintError("save_corr_for_debugging\n");
    std::ofstream myfile;
    myfile.open(file_name);
    int line_cnt = 0;
    for (auto c : corr) {
        int lid0 = c.source_point_.scan_line_id_;
        int vid0 = c.source_point_.vertex_id_;
        int lid1 = c.target_points_[i_th_feature].scan_line_id_;
        int vid1 = c.target_points_[i_th_feature].vertex_id_;
        myfile << std::to_string(lid0 * 870 + vid0) << " "
               << std::to_string(lid1 * 870 + vid1) << "\n";
    }
    myfile.close();
}

void save_pose_for_debugging(Eigen::Matrix4d trans,
        const char* file_name)
{
    PrintError("save_pose_for_debugging\n");
    std::ofstream myfile;
    myfile.open(file_name);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            myfile << trans(i, j) << " ";
        }
        myfile << std::endl;
    }
    myfile.close();
}

std::tuple<bool, Eigen::Matrix4d> Execution(const LidarScan &source_input,
                                            const LidarScan &target,
                                            const OdometryLidarOption &option) {
    // need to do some warping for source point cloud.
    LidarScan source = source_input;
    auto source_index = Preprocessing(source, true, option);
    auto target_index = Preprocessing(target, true, option);
    auto target_tree = MakePointTrees(target,
            std::get<0>(target_index), std::get<1>(target_index));

    // for debugging
    // save_feature_for_debugging(std::get<0>(source_index), "source_edge.txt");
    // save_feature_for_debugging(std::get<1>(source_index), "source_planar.txt");
    // save_feature_for_debugging(std::get<0>(target_index), "target_edge.txt");
    // save_feature_for_debugging(std::get<1>(target_index), "target_planar.txt");

    bool is_success = false;
    Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();

    for (int iter = 0; iter < option.iteration_number_; iter++) {

        PrintDebug("Iteration : %d / %d\n", iter, option.iteration_number_);

        auto edge_corr = GetMatchingPoints(
                std::get<0>(source_index), std::get<0>(target_index),
                source, std::get<0>(target_tree), EDGE);

        auto planar_corr = GetMatchingPoints(
                std::get<1>(source_index), std::get<1>(target_index),
                source, std::get<1>(target_tree), PLANAR);

        // save_corr_for_debugging(edge_corr, "edge_corr_0.txt", 0);
        // save_corr_for_debugging(edge_corr, "edge_corr_1.txt", 1);
        // save_corr_for_debugging(planar_corr, "planar_corr_0.txt", 0);
        // save_corr_for_debugging(planar_corr, "planar_corr_1.txt", 1);
        // save_corr_for_debugging(planar_corr, "planar_corr_2.txt", 2);

        // retrieve 6D motion
        auto f_edge = [&](int i, Eigen::Vector6d &J_r, double &r) {
            ComputeJacobianAndResidualForEdgeFeatures(i, J_r, r, source, target, edge_corr);
        };
        auto f_planar = [&](int i, Eigen::Vector6d &J_r, double &r) {
            ComputeJacobianAndResidualForPlanarFeatures(i, J_r, r, source, target, planar_corr);
        };

        // PrintDebug("Iter : %d\n", iter);
        Eigen::Matrix6d JTJ, JTJ_edge, JTJ_planar;
        Eigen::Vector6d JTr, JTr_edge, JTr_planar;
        std::tie(JTJ_edge, JTr_edge) = ComputeJTJandJTr<Eigen::Matrix6d, Eigen::Vector6d>(
                f_edge, edge_corr.size());
        std::tie(JTJ_planar, JTr_planar) = ComputeJTJandJTr<Eigen::Matrix6d, Eigen::Vector6d>(
                f_planar, planar_corr.size());
        JTJ = JTJ_edge + JTJ_planar;
        JTr = JTr_edge + JTr_planar;
        // std::cout << "JTJ_edge" << std::endl;
        // std::cout << JTJ_edge << std::endl;
        // std::cout << "JTr_edge" << std::endl;
        // std::cout << JTr_edge << std::endl;
        // std::cout << "JTJ_planar" << std::endl;
        // std::cout << JTJ_planar << std::endl;
        // std::cout << "JTr_planar" << std::endl;
        // std::cout << JTr_planar << std::endl;

        Eigen::Matrix4d delta;
        std::tie(is_success, delta) =
                SolveJacobianSystemAndObtainExtrinsicMatrix(JTJ, JTr);
        // std::cout << delta << std::endl;

        // std::cout << "Estimated 4x4 motion matrix : " << std::endl;
        source.Transform(delta);
        extrinsic = delta * extrinsic;
    }

    save_pose_for_debugging(extrinsic, "pose.txt");

        if (!is_success) {
        PrintWarning("[ComputeOdometryLidar] no solution!\n");
        return std::make_tuple(false, Eigen::Matrix4d::Identity());
    } else {
        return std::make_tuple(true, extrinsic);
    }

    return std::make_tuple(false, Eigen::Matrix4d::Identity());
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