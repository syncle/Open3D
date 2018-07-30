// #include "StdAfx.h"
#include "PointCloudFragment.h"
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
//
PointCloudFragment::PointCloudFragment(int index, int resolution, float length)
{
    resolution_ = resolution;
    length_ = length;
    unit_length_ = length / resolution;
    index_ = index;
    nper_ = (resolution_ + 1) * (resolution_ + 1) * (resolution_ + 1) * 3;
    offset_ = index * nper_;
}

PointCloudFragment::~PointCloudFragment(void)
{
}
//
void PointCloudFragment::LoadFromPCDFile(const char * filename)
{
//     pcl::PointCloudFragment<pcl::PointXYZRGBNormal>::Ptr rawpcd(new pcl::PointCloudFragment<pcl::PointXYZRGBNormal>);
//     if (pcl::io::loadPCDFile(filename, *rawpcd) < 0) {
//         PCL_ERROR("Error loading file.\n");
//         return;
//     }
//     float x[6];
//     for (int i = 0; i < (int)rawpcd->points.size(); i++) {
//         if (!_isnan(rawpcd->points[i].normal_x)) {
//             points_.resize(points_.size() + 1);
//             x[0] = rawpcd->points[i].x;
//             x[1] = rawpcd->points[i].y;
//             x[2] = rawpcd->points[i].z;
//             x[3] = rawpcd->points[i].normal_x;
//             x[4] = rawpcd->points[i].normal_y;
//             x[5] = rawpcd->points[i].normal_z;
//             if (GetCoordinate(x, points_.back()) == false) {
//                 PCL_ERROR("Error!! Point out of bound!!\n");
//                 return;
//             }
//             points_.back().rgb_[0] = rawpcd->points[i].r;
//             points_.back().rgb_[1] = rawpcd->points[i].g;
//             points_.back().rgb_[2] = rawpcd->points[i].b;
//         }
    // }
//    printf("Read %s ... get %d points.\n", filename, points_.size());
}

void PointCloudFragment::LoadFromPLYFile(const char * filename)
{
//     pcl::PointCloudFragment<pcl::PointXYZRGBNormal>::Ptr rawpcd(new pcl::PointCloudFragment<pcl::PointXYZRGBNormal>);
//     if (pcl::io::loadPLYFile(filename, *rawpcd) < 0) {
//         PCL_ERROR("Error loading file.\n");
//         return;
//     }
//     float x[6];
//     for (int i = 0; i < (int)rawpcd->points.size(); i++) {
//         if (!_isnan(rawpcd->points[i].normal_x)) {
//             points_.resize(points_.size() + 1);
//             x[0] = rawpcd->points[i].x;
//             x[1] = rawpcd->points[i].y;
//             x[2] = rawpcd->points[i].z;
//             x[3] = rawpcd->points[i].normal_x;
//             x[4] = rawpcd->points[i].normal_y;
//             x[5] = rawpcd->points[i].normal_z;
//             if (GetCoordinate(x, points_.back()) == false) {
//                 PCL_ERROR("Error!! Point out of bound!!\n");
//                 return;
//             }
//             points_.back().rgb_[0] = rawpcd->points[i].r;
//             points_.back().rgb_[1] = rawpcd->points[i].g;
//             points_.back().rgb_[2] = rawpcd->points[i].b;
//         }
//     }
//    printf("Read %s ... get %d points.\n", filename, points_.size());
    three::PointCloud pcd;
    ReadPointCloud(filename, pcd);
    float x[6];
    for (auto i = 0; i < pcd.points_.size(); i++) {
        if (!isnan(pcd.normals_[i][0])) {
            points_.resize(points_.size() + 1);
            x[0] = pcd.points_[i][0];
            x[1] = pcd.points_[i][1];
            x[2] = pcd.points_[i][2];
            x[3] = pcd.normals_[i][0];
            x[4] = pcd.normals_[i][1];
            x[5] = pcd.normals_[i][2];
            if (GetCoordinate(x, points_.back()) == false) {
                three::PrintError("Error!! Point out of bound!!\n");
                return;
            }
            // double to uchar?
            points_.back().rgb_[0] = pcd.colors_[i][0];
            points_.back().rgb_[1] = pcd.colors_[i][1];
            points_.back().rgb_[2] = pcd.colors_[i][2];
        }
    }
    three::PrintInfo("Read %s ... get %d points.\n",
            filename, (int)points_.size());
}

void PointCloudFragment::LoadFromXYZNFile(const char * filename)
{
//     FILE * f = fopen(filename, "r");
//     if (f != NULL) {
//         char buffer[1024];
//         float x[6];
//         while (fgets(buffer, 1024, f) != NULL) {
//             if (strlen(buffer) > 0 && buffer[0] != '#') {
//                 sscanf(buffer, "%f %f %f %f %f %f", &x[0], &x[1], &x[2], &x[3], &x[4], &x[5]);
//                 points_.resize(points_.size() + 1);
//                 if (GetCoordinate(x, points_.back()) == false) {
//                     PCL_ERROR("Error!! Point out of bound!!\n");
//                     return;
//                 }
//             }
//         }
//         fclose (f);
//         printf("Read %s ... get %d points.\n", filename, points_.size());
//    }
}
