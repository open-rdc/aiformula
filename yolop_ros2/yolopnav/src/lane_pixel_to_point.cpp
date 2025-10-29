#include "yolopnav/lane_pixel_to_point.hpp"
#include <cmath>

namespace yolopnav {

LanePixelToPoint::LanePixelToPoint() {
    // Pre-compute camera matrix and its inverse once in constructor
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_32F);
    camera_matrix.at<float>(0, 0) = CameraIntrinsics::fx;
    camera_matrix.at<float>(1, 1) = CameraIntrinsics::fy;
    camera_matrix.at<float>(0, 2) = CameraIntrinsics::cx;
    camera_matrix.at<float>(1, 2) = CameraIntrinsics::cy;

    inv_camera_matrix_ = camera_matrix.inv();
}

Eigen::Vector3d LanePixelToPoint::pixelToRobotPoint(const cv::Point& pixel) const {
    // Transform pixel to camera vector using pre-computed inverse camera matrix
    cv::Mat pixel_matrix = (cv::Mat_<float>(3, 1) << pixel.x, pixel.y, 1.0);
    cv::Mat camera_vec_matrix = inv_camera_matrix_ * pixel_matrix;
    
    double x_cam = camera_vec_matrix.at<float>(0, 0);
    double y_cam = camera_vec_matrix.at<float>(1, 0);
    double z_cam = camera_vec_matrix.at<float>(2, 0);
    
    // For horizontal camera: ground plane projection using y_cam
    // Camera height above ground, bottom of image shows ground (y_cam > 0)
    double scale = CameraIntrinsics::camera_height / y_cam;
    
    // Skip points above horizon (y_cam <= 0) or invalid scale
    if (y_cam <= 0.0 || scale <= 0.0) {
        return Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    
    double x_ground = x_cam * scale;
    double z_ground = z_cam * scale;

    double x_robot = z_ground;
    double y_robot = -x_ground;
    double z_robot = 0.0;
    
    return Eigen::Vector3d(x_robot, y_robot, z_robot);
}

std::vector<Eigen::Vector3d> LanePixelToPoint::pixelsToRobotPoints(const std::vector<cv::Point>& pixels) const {
    std::vector<Eigen::Vector3d> robot_points;
    robot_points.reserve(pixels.size());
    
    for (const auto& pixel : pixels) {
        Eigen::Vector3d robot_point = pixelToRobotPoint(pixel);
        // Only add valid transformations (non-zero points from invalid scale) - matching aiformula approach
        if (robot_point.x() != 0.0 || robot_point.y() != 0.0 || robot_point.z() != 0.0) {
            robot_points.push_back(robot_point);
        }
    }
    
    return robot_points;
}

}  // namespace yolopnav