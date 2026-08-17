#include "camera_utility/camera_intrinsics.hpp"

#include <stdexcept>

namespace camera_utility
{

CameraIntrinsics getCameraIntrinsics(const std::string& resolution)
{
    if (resolution == "nHD") {
        return CameraIntrinsics{640, 360, 254.391622, 254.391622, 330.013020833, 181.149637858};
    }
    if (resolution == "SVGA") {
        return CameraIntrinsics{
            960, 600, 377.3742370605469, 377.3742370605469, 495.00592041015625, 301.7193908691406};
    }
    throw std::invalid_argument("未対応のカメラ解像度: " + resolution);
}

}
