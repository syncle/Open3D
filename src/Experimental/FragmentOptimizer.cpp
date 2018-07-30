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

#include <Core/Core.h>
#include <Core/Registration/FragmentOptimizer.h>
#include <Core/Registration/FragmentOptimizerOption.h>
#include <Core/Utility/Console.h>

int PrintHelp(char* argv[])
{
    using namespace three;

    PrintInfo("Open3D %s\n", OPEN3D_VERSION);
    PrintInfo("\n");
    PrintInfo("Usage:\n");
    PrintInfo("> %s [options]\n\n", argv[0]);

    PrintInfo("Application parameters:\n");
    PrintInfo("    --help, -h                      : print this message\n");
    PrintInfo("    --rgbdslam <log_file>           : rgbdslam.log/opt_output.log file, get ipose\n");
    PrintInfo("    --registration <log_file>       : reg_output.log, invalid pair when frame_ == -1\n");
    PrintInfo("    --dir <dir_prefix>              : dir prefix, place to loopup .xyzn files\n");
    PrintInfo("    --num <number>                  : number of pieces, important parameter\n");
    PrintInfo("    --weight <weight>               : 1.0 for nonrigid, 10000.0 for rigid\n");
    PrintInfo("    --resolution <resolution>       : default - 8\n");
    PrintInfo("    --length <length>               : default - 3.0\n");
    PrintInfo("    --interval <interval>           : default - 50\n");
    PrintInfo("    --iteration <max_number>        : default - 5\n");
    PrintInfo("    --inner_iteration <max_number>  : default - 10\n");
    PrintInfo("    --save_to <ctr_file>            : default - output.ctr\n");
    PrintInfo("    --write_xyzn_sample <sample_num>: per <sample_num> write a point\n");
    PrintInfo("    --blasklist <blacklist_file>    : each line is the block we want to blacklist\n");
    PrintInfo("    --blacklistpair <threshold>     : threshold of accepting pairwise registration, default - 10000\n");
    PrintInfo("    --switchable <weight>           : use switchable g2o, default weight 0.1\n");
    PrintInfo("    --ipose <log_file>              : get ipose from log file\n");
    PrintInfo("    --verbose <verbpse_level>       : set verbosity level, from 0(none) to 5(verbose), default 3\n");
    PrintInfo("Optimization options:\n");
    PrintInfo("    --rigid                         : dense rigid optimization\n");
    PrintInfo("    --rigid_p2p                     : dense rigid optimization with point to point distance\n");
    PrintInfo("    --nonrigid                      : default, nonrigid alignment published in ICCV 2013\n");
    PrintInfo("    --slac                          : simultaneous localization and calibration, published in CVPR 2014\n");
    PrintInfo("    --slac_p2p                      : simultaneous localization and calibration, with point to point distance\n");
    return 0;
}

int main(int argc, char *argv[])
{
    using namespace three;

    if (ProgramOptionExists(argc, argv, "--help")) {
        PrintHelp(argv);
        return 0;
    }
    FragmentOptimizerOption option;
    if (ProgramOptionExists(argc, argv, "--rgbdslam")) {
        option.rgbd_filename_ = GetProgramOptionAsString(
                argc, argv, "--rgbdslam");
    }
    if (ProgramOptionExists(argc, argv, "--dir")) {
        option.dir_prefix_ = GetProgramOptionAsString(
            argc, argv, "--dir");
    }
    if (ProgramOptionExists(argc, argv, "--registration")) {
        option.reg_filename_ = GetProgramOptionAsString(
                argc, argv, "--registration");
    }
    if (ProgramOptionExists(argc, argv, "--num")) {
        option.num_ = GetProgramOptionAsInt(
            argc, argv, "--num");
    }
    if (ProgramOptionExists(argc, argv, "--resolution")) {
        option.resolution_ = GetProgramOptionAsInt(
            argc, argv, "--resolution");
    }
    if (ProgramOptionExists(argc, argv, "--length")) {
        option.length_ = GetProgramOptionAsDouble(
            argc, argv, "--length");
    }
    if (ProgramOptionExists(argc, argv, "--iteration")) {
        option.max_iteration_ = GetProgramOptionAsInt(
            argc, argv, "--iteration");
    }
    // if (blacklist_file = GetProgramOptionAsString(argc, argv, "--blacklist", blacklist_file) > 0) {
        // Blacklist(blacklist_file);
    // }
    // if (ipose_file = GetProgramOptionAsString(argc, argv, "--ipose", ipose_file)) {
        // IPoseFromFile(ipose_file);
    // }
    if (ProgramOptionExists(argc, argv, "--slac")) {
        option.type_ = FragmentOptimizerType::SLAC;
    } else if (ProgramOptionExists(argc, argv, "--rigid")) {
        option.type_ = FragmentOptimizerType::Rigid;
    } else if (ProgramOptionExists(argc, argv, "--rigid_p2p")) {
        option.type_ = FragmentOptimizerType::RigidP2P;
    } else if (ProgramOptionExists(argc, argv, "--slac_p2p")) {
        option.type_ = FragmentOptimizerType::SLACP2P;
    } else if (ProgramOptionExists(argc, argv, "--nonrigid")) {
        option.type_ = FragmentOptimizerType::Nonrigid;
    }
    FragmentOptimizer(option);
}
