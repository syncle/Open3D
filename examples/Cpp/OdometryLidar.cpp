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

#include <iostream>
#include <memory>
#include <Core/Core.h>
#include <Core/Odometry/OdometryLidar.h>
#include <Core/Odometry/OdometryLidarOption.h>
#include <Core/Utility/Console.h>
#include <IO/ClassIO/LidarScanIO.h>
#include <IO/ClassIO/PointCloudIO.h>
#include <Visualization/Visualization.h>

// #include <IO/IO.h>

void PrintHelp(char* argv[])
{
    using namespace open3d;

    PrintOpen3DVersion();
    PrintInfo("Usage:\n");
    PrintInfo(">    OdometryLidar [source] [target] [options]\n");
    PrintInfo("     Given depth image pair of Lidar scan, estimate 6D odometry.\n");
    PrintInfo("     [options]\n");
    PrintInfo("\n");
}

int main(int argc, char *argv[])
{
    using namespace open3d;

    if (argc <= 2 || ProgramOptionExists(argc, argv, "--help") ||
            ProgramOptionExists(argc, argv, "-h")) {
        PrintHelp(argv);
        return 1;
    }

    if (ProgramOptionExists(argc, argv, "--verbose"))
        SetVerbosityLevel(VerbosityLevel::VerboseAlways);

    auto source = ReadLidarScanFromKITFormat(argv[1]);
    auto target = ReadLidarScanFromKITFormat(argv[2]);

    // // visualize scanned data
    auto source_pcd = CreatePointCloudFromLidarScan(*source);
    auto target_pcd = CreatePointCloudFromLidarScan(*target);
    DrawGeometries({source_pcd, target_pcd});
    WritePointCloud("source.ply", *source_pcd);
    WritePointCloud("target.ply", *target_pcd);

    // compute odometry from Lidar data
    OdometryLidarOption option;
    bool is_success;
    Eigen::Matrix4d trans;
    std::tie(is_success, trans) = ComputeOdometryLidar(
        *source, *target, Eigen::Matrix4d::Identity(), option);

    std::cout << "Estimated 4x4 motion matrix : " << std::endl;
    std::cout << trans << std::endl;

    return int(!is_success);
}
