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

namespace three {

enum class FragmentOptimizerType {
    SLAC = 0,
    Rigid = 1,
    RigidP2P = 2,
    SLACP2P = 3,
    Nonrigid = 4,
};

class FragmentOptimizerOption
{
public:
    FragmentOptimizerOption(
            int resolution = 8, int interval = 50, int num = 0,
            double weight = 1.0, double length = 3.0, int max_iteration = 5,
            int max_inner_iteration = 10,
            std::string ctr_filename = "output.ctr",
            std::string sample_filename = "sample.pcd",
            std::string pose_filename = "pose.log",
            std::string init_ctr_file = "",
            std::string dir_prefix = "",
            int sample_num = -1,
            int blacklist_pair_num = 10000,
            FragmentOptimizerType type = FragmentOptimizerType::SLACP2P) :
            resolution_(resolution),
            interval_(interval),
            num_(num),
            weight_(weight),
            length_(length),
            max_iteration_(max_iteration),
            max_inner_iteration_(max_inner_iteration),
            ctr_filename_(ctr_filename),
            sample_filename_(sample_filename),
            pose_filename_(pose_filename),
            init_ctr_file_(init_ctr_file),
            dir_prefix_(dir_prefix),
            sample_num_(sample_num),
            blacklist_pair_num_(blacklist_pair_num),
            type_(type) {}
    ~FragmentOptimizerOption() {}

public:
    int resolution_;
    int interval_;
    int num_;
    double weight_;
    double length_;
    int max_iteration_;
    int max_inner_iteration_;
    std::string dir_prefix_;
    std::string rgbd_filename_;
    std::string reg_filename_;
    std::string ctr_filename_;
    std::string sample_filename_;
    std::string init_ctr_file_;
    std::string pose_filename_;
    int sample_num_;
    int blacklist_pair_num_;
    FragmentOptimizerType type_;
};

}
