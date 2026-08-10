#pragma once

#include <string>

namespace camera_utility
{

struct CameraIntrinsics
{
    int width;
    int height;
    double fx;
    double fy;
    double cx;
    double cy;
};

CameraIntrinsics getCameraIntrinsics(const std::string& resolution);

}
