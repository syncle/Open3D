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

#include <cstdlib>
#include <ctime>
#include <random>
#include <limits>
#include <vector>

#include <Core/Utility/Console.h>
#include <Core/Geometry/PointCloud.h>
#include <Core/Registration/Registration.h>
#include <Core/Registration/Feature.h>
#include <IO/ClassIO/PointCloudIO.h>

using namespace three;

void TestRandomSeed() {
    //////////////////////////////
    // Random seed test
    //////////////////////////////
    const int N_BIN = 100;
    thread_local static std::mt19937 engine(std::random_device{}());
    std::uniform_int_distribution<int> uniform_dist(
        0, std::numeric_limits<int>::max());
    std::vector<int> bin1(N_BIN);
    std::vector<int> bin2(N_BIN);
    for (int i = 0; i < N_BIN * 1000; i++) {
        bin1[uniform_dist(engine) % N_BIN]++;
        bin2[rand() % N_BIN]++;
    }
    for (int i = 0; i < N_BIN; i++) {
        PrintInfo("bin1[%d] : %d\n", i, bin1[i]);
    }
    for (int i = 0; i < N_BIN; i++) {
        PrintInfo("bin2[%d] : %d\n", i, bin2[i]);
    }
}

std::pair<std::shared_ptr<PointCloud>, std::shared_ptr<Feature>>
        PreprocessPointCloud(
        std::shared_ptr<PointCloud> pcd, const double voxel_size) {
    PrintInfo(":: Downsample with a voxel size %.3f.\n", voxel_size);
    auto pcd_down = VoxelDownSample(*pcd, voxel_size);

    const double radius_normal = voxel_size * 2.0;
    PrintInfo(":: Estimate normal with search radius %.3f.\n", radius_normal);
    EstimateNormals(*pcd_down, KDTreeSearchParamHybrid(radius_normal, 30));

    const double radius_feature = voxel_size * 5.0;
    PrintInfo(":: Compute FPFH feature with search radius %.3f.\n", radius_feature);
    auto pcd_fpfh = ComputeFPFHFeature(*pcd_down,
        KDTreeSearchParamHybrid(radius_feature, 100));
    return std::make_pair(pcd_down, pcd_fpfh);
}

int main(int argc, char **argv)
{
    TestRandomSeed();
    std::shared_ptr<PointCloud> source, target;
    source = CreatePointCloudFromFile("C:/git/Open3D/build/lib/Release/Tutorial/Benchmark/testdata/livingroom2/cloud_bin_25.ply");
    target = CreatePointCloudFromFile("C:/git/Open3D/build/lib/Release/Tutorial/Benchmark/testdata/livingroom2/cloud_bin_41.ply");
    std::shared_ptr<PointCloud> source_down, target_down;
    std::shared_ptr<Feature> source_fpfh, target_fpfh;
    const double voxel_size = 0.05;
    const double distance_threshold = voxel_size * 1.5;
    std::tie(source_down, source_fpfh) = PreprocessPointCloud(source, voxel_size);
    std::tie(target_down, target_fpfh) = PreprocessPointCloud(target, voxel_size);
    std::vector<std::reference_wrapper<const CorrespondenceChecker>>
            correspondence_checker;
    auto correspondence_checker_edge_length =
        CorrespondenceCheckerBasedOnEdgeLength(0.9);
    auto correspondence_checker_distance =
        CorrespondenceCheckerBasedOnDistance(0.075);
    correspondence_checker.push_back(correspondence_checker_edge_length);
    correspondence_checker.push_back(correspondence_checker_distance);
    for (int i = 0; i < 1000000; i++) {
        PrintInfo("Stress test %d\n", i);
        RegistrationRANSACBasedOnFeatureMatching(*source_down, *target_down,
                *source_fpfh, *target_fpfh, distance_threshold,
                TransformationEstimationPointToPoint(false), 4,
                correspondence_checker,
                RANSACConvergenceCriteria(4000000, 500));
    }
}
