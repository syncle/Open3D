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

#include "LidarScan.h"

// #include <Eigen/Dense>
#include <Core/Utility/Console.h>           // just for debugging
// #include <Core/Geometry/KDTreeFlann.h>
#include <Core/Geometry/PointCloud.h>

namespace open3d{

void LidarScan::Clear()
{
    scan_lines_.clear();
}

bool LidarScan::IsEmpty() // const?
{
    // return !HasPoints();
    return true;
}

bool LidarScan::Transform(Eigen::Matrix4d T /*=Eigen::Matrix4d::Identity()*/) {
    for (auto &line : scan_lines_) {
        for (auto &point : line.points_) {
            Eigen::Vector4d new_point = T * Eigen::Vector4d(
                point(0), point(1), point(2), 1.0);
            point = new_point.block<3, 1>(0, 0);
        }
    }
    return true;
}

bool LidarScan::UndistortScan(Eigen::Matrix4d T /*=Eigen::Matrix4d::Identity()*/) {
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);
    for (auto &line : scan_lines_) {
        for (int i = 0; i < line.points_per_line_; i++) {
            double ratio = double(i) / (line.points_per_line_ - 1);
            // linear motion model
            line.points_[i] = ratio * R * line.points_[i] + ratio * t;
        }
    }
    return true;
}

std::shared_ptr<PointCloud> CreatePointCloudFromLidarScan(
        const LidarScan& scan)
{
    auto pcd = std::make_shared<PointCloud>();
    for (auto line : scan.scan_lines_) {
        for (int i=0; i<line.points_per_line_; i++) {
            auto p = line.points_[i];
            pcd->points_.push_back(p);
        }
    }
    return pcd;
}

}    // namespace open3d
