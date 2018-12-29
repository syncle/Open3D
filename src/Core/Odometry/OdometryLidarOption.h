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

#include <string>

namespace open3d {

class OdometryLidarOption
{
public:
    OdometryLidarOption(
            double feature_selection_threshold = 5e-2,
            double angular_threshold = 0.8660,/*approx. cos(30deg)*/
            double depth_diff_threshold = 3.0,/*in meter*/
            int region_div_ = 16,
            int edge_feature_in_div = 4,
            int planar_feature_in_div = 4,
            int curvature_window_half_size = 5) :
            feature_selection_threshold_(feature_selection_threshold),
            angular_threshold_(angular_threshold),
            depth_diff_threshold_(depth_diff_threshold),
            region_div_(region_div_),
            edge_feature_in_div_(edge_feature_in_div),
            planar_feature_in_div_(planar_feature_in_div),
            curvature_window_half_size_(curvature_window_half_size) {}
    ~OdometryLidarOption() {}

public:
    double feature_selection_threshold_;
    double angular_threshold_;
    double depth_diff_threshold_;
    int region_div_; // division number to distribute features evenly
    int edge_feature_in_div_;
    int planar_feature_in_div_;
    int curvature_window_half_size_;
};

}
