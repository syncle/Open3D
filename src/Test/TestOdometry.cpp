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

#include <cstdio>

#include <Core/Core.h>
#include <Core/Odometry/Odometry.h>
#include <Core/Utility/Console.h>
#include <IO/IO.h>

int main(int argc, char **argv)
{
	using namespace three;

	SetVerbosityLevel(VerbosityLevel::VerboseAlways);

	//if (argc != 3) {
	//	PrintInfo("Open3D %s\n", OPEN3D_VERSION);
	//	PrintInfo("\n");
	//	PrintInfo("Usage:\n");
	//	PrintInfo("    > TestImage [image filename] [depth filename]\n");
	//	PrintInfo("    The program will :\n");
	//	PrintInfo("    1) Read 8bit RGB and 16bit depth image\n");
	//	PrintInfo("    2) Convert RGB image to single channel float image\n");
	//	PrintInfo("    3) 3x3, 5x5, 7x7 Gaussian filters are applied\n");
	//	PrintInfo("    4) 3x3 Sobel filter for x-and-y-directions are applied\n");
	//	PrintInfo("    5) Make image pyramid that includes Gaussian blur and downsampling\n");
	//	PrintInfo("    6) Will save all the layers in the image pyramid\n");
	//	return 0;
	//}

	const std::string filename_rgb1(argv[1]);
    const std::string filename_rgb2(argv[2]);
	const std::string filename_depth1(argv[3]);
    const std::string filename_depth2(argv[4]);

	Image color_image_8bit1, color_image_8bit2;
    Image depth_image_16bit1, depth_image_16bit2;
    ReadImage(filename_rgb1, color_image_8bit1);
    ReadImage(filename_rgb2, color_image_8bit2);
    ReadImage(filename_depth1, depth_image_16bit1);
    ReadImage(filename_depth2, depth_image_16bit2);

    auto rgbd_image1 = CreateRGBDImageFromRedwoodFormat(color_image_8bit1, depth_image_16bit1);
    auto rgbd_image2 = CreateRGBDImageFromRedwoodFormat(color_image_8bit2, depth_image_16bit2);

    bool success;
    Eigen::Matrix4d trans;
    Eigen::Matrix6d info;
    PinholeCameraIntrinsic intrinsic = PinholeCameraIntrinsicParameters::PrimeSenseDefault;
    auto jacobian = RGBDOdometryJacobianFromHybridTerm();
    Eigen::Matrix4d init = Eigen::Matrix4d::Identity();
    OdometryOption option;
    std::tie(success, trans, info) = ComputeRGBDOdometry(*rgbd_image1, *rgbd_image2, intrinsic, init, jacobian, option);

    //PrintDebug("%.3f %.3f %.3f %.3f\n %.3f %.3f %.3f %.3f\n %.3f %.3f %.3f %.3f\n %.3f %.3f %.3f %.3f\n",
    //        trans(0)(0), trans[0][0], trans[0][0], trans[0][0],
    //        trans[0][0], trans[0][0], trans[0][0], trans[0][0],
    //        trans[0][0], trans[0][0], trans[0][0], trans[0][0],
    //        trans[0][0], trans[0][0], trans[0][0], trans[0][0]);

	return 0;
}
