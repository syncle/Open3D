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

#include <math.h>
#include <Core/Geometry/Image.h>
#include <Core/Geometry/LidarScan.h>
#include <IO/ClassIO/ImageIO.h>

// #define M_PI 3.14159265358979323846

namespace open3d {

namespace {

double velodyne_hd64_vertical[64] = {
    -1.9367,  -1.57397, -1.30476, -0.871566, -0.57881, -0.180617, 0.088762,
    0.451829, 0.80315,  1.20124,  1.49388,   1.83324,  2.20757,   2.54663,
    2.87384,  3.23588,  3.53933,  3.93585,   4.21552,  4.5881,    4.91379,
    5.25078,  5.6106,   5.9584,   6.32889,   6.67575,  6.99904,   7.28731,
    7.67877,  8.05803,  8.31047,  8.71141,   9.02602,  9.57351,   10.0625,
    10.4707,  10.9569,  11.599,   12.115,    12.5621,  13.041,    13.4848,
    14.0483,  14.5981,  15.1887,  15.6567,   16.1766,  16.554,    17.1868,
    17.7304,  18.3234,  18.7971,  19.3202,   19.7364,  20.2226,   20.7877,
    21.3181,  21.9355,  22.4376,  22.8566,   23.3224,  23.971,    24.5066,
    24.9992};

inline double radian(double input)
{
    return input / 180.0 * M_PI;
}

}   // unnamed namespace

std::shared_ptr<LidarScan> ReadLidarScanFromKITFormat(
    const std::string &depth_filename, LidarScan &trajectory) {
    // read depth image
    auto scan = std::make_shared<LidarScan>();
    auto depth = CreateImageFromFile("depth_filename");
    // std::assert(depth.height_ != 64);
    double scale = -360.0 / depth->width_;
    for (int i = 0; i < depth->height_; i++) {
        LidarScanLine line;
        for (int j = 0; j < depth->width_; j++) {
            float d = *PointerAt<unsigned short>(*depth, j, i);
            float theta = radian(double(j) * scale + 180);
            float rho = radian(velodyne_hd64_vertical[i]); // not necessary
            float x = d * std::sin(rho) * std::cos(theta);
            float y = d * std::sin(rho) * std::sin(theta);
            float z = d * std::cos(rho);
            Eigen::Vector3d p;
            p << x, y, z;
            line.points_.push_back(p);
        }
        scan->scan_lines_.push_back(line);
    }
    return scan;
}

}  // namespace open3d