#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <ceres/rotation.h>

#include "booster_vision/base/intrin.h"

namespace booster_vision
{
    template <typename T>
    void Project(const T *const point, const T *const intr, const T *const d, T *uv)
    {
        T fx = intr[0];
        T fy = intr[1];
        T cx = intr[2];
        T cy = intr[3];

        T x = point[0] / point[2];
        T y = point[1] / point[2];

        T r2 = x * x + y * y;
        T r4 = r2 * r2;
        T r6 = r4 * r2;

        T x_prime = x * (T(1) + d[0] * r2 + d[1] * r4 + d[4] * r6) + T(2) * d[2] * x * y + d[3] * (r2 + T(2) * x * x);
        T y_prime = y * (T(1) + d[0] * r2 + d[1] * r4 + d[4] * r6) + d[2] * (r2 + T(2) * y * y) + T(2) * d[3] * x * y;

        uv[0] = fx * x_prime + cx;
        uv[1] = fy * y_prime + cy;
    }

    class ExtrinsicsOptimizer
    {
    public:
        ExtrinsicsOptimizer(const cv::Point3f &object_point, const cv::Point2f &uv,
                            const Eigen::Quaterniond &q_base2head, const Eigen::Vector3d &t_base2head,
                            const Intrinsics &intr) : object_point_(object_point),
                                                      uv_(uv), q_base2head_(q_base2head), t_base2head_(t_base2head), intr_(intr)
        {
        }

        template <typename T>
        bool operator()(const T *const q_head2cam_data, const T *const t_head2cam_data,
                        const T *const q_board2base_data, const T *const t_board2base_data,
                        T *residuals) const
        {
            Eigen::Quaternion<T> q_head2cam = Eigen::Quaternion<T>(q_head2cam_data);
            Eigen::Quaternion<T> q_base2cam = q_head2cam * q_base2head_.cast<T>();
            Eigen::Matrix<T, 3, 1> t_base2cam = q_head2cam * Eigen::Matrix<T, 3, 1>(t_base2head_.cast<T>()) + Eigen::Matrix<T, 3, 1>(t_head2cam_data);

            Eigen::Quaternion<T> q_board2base = Eigen::Quaternion<T>(q_board2base_data);
            Eigen::Quaternion<T> q_board2cam = q_base2cam * q_board2base;
            Eigen::Matrix<T, 3, 1> t_board2cam = q_base2cam * Eigen::Matrix<T, 3, 1>(t_board2base_data) + t_base2cam;

            T obj[3] = {T(object_point_.x), T(object_point_.y), T(object_point_.z)};
            T q[4] = {q_board2cam.w(), q_board2cam.x(), q_board2cam.y(), q_board2cam.z()};
            T obj_cam[3];

            ceres::QuaternionRotatePoint(q, obj, obj_cam);
            obj_cam[0] += t_board2cam[0];
            obj_cam[1] += t_board2cam[1];
            obj_cam[2] += t_board2cam[2];

            T uv[2];
            T intr[4] = {T(intr_.fx), T(intr_.fy), T(intr_.cx), T(intr_.cy)};
            if (intr_.distortion_coeffs.empty())
            {
                T d[5] = {T(0),T(0),T(0),T(0),T(0)};
                Project(obj_cam, intr, d, uv);
            }
            else
            {
                T d[5] = {T(intr_.distortion_coeffs[0]), T(intr_.distortion_coeffs[1]),
                          T(intr_.distortion_coeffs[2]), T(intr_.distortion_coeffs[3]),
                          T(intr_.distortion_coeffs[4])};
                Project(obj_cam, intr, d, uv);
            }

            residuals[0] = uv[0] - T(uv_.x);
            residuals[1] = uv[1] - T(uv_.y);
            return true;
        }

    private:
        cv::Point3f object_point_;
        cv::Point2f uv_;
        Eigen::Quaterniond q_base2head_;
        Eigen::Vector3d t_base2head_;
        Intrinsics intr_;
    };

} // namespace booster_vision
