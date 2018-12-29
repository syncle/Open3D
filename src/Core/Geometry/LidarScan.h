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

#pragma once

#include <vector>
#include <memory>
#include <Core/Geometry/Geometry3D.h>

namespace open3d {

class PointCloud;

class LidarScanLine {
public:
    int points_per_line_;
    std::vector<bool> is_valid_;
    std::vector<Eigen::Vector3d> points_; // should be the order of the scanning
    std::vector<Eigen::Vector3d> colors_;
};

class LidarScan {
public:
    std::vector<LidarScanLine> scan_lines_;
public:
    void Clear();
    bool IsEmpty();
    bool UndistortScan(Eigen::Matrix4d T = Eigen::Matrix4d::Identity());
};

class LiderScanPoint {
public:
    int scan_line_id_;
    int vertex_id_;
};

std::shared_ptr<PointCloud> CreatePointCloudFromLidarScan(const LidarScan& scan);

}   // namespace open3d
