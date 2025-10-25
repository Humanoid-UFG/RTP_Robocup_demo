#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "booster_vision/base/intrin.h"
#include "booster_vision/base/pose.h"

namespace booster_vision {

void EyeInHandCalibration(double *reprojection_error,
                          Pose *p_cam2head,
                          Pose *p_board2base,
                          const std::vector<Pose> &p_board2cameras,
                          const std::vector<Pose> &p_head2bases,
                          const std::vector<std::vector<cv::Point3f>> &all_corners_3d,
                          const std::vector<std::vector<cv::Point2f>> &all_corners_2d,
                          const Intrinsics &intr,
                          bool optimization = true);

} // namespace booster_vision