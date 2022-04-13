/*
 * @Description:
 * @Version: 2.0
 * @Author: C-Xingyu
 * @Date: 2022-03-25 16:20:47
 * @LastEditors: C-Xingyu
 * @LastEditTime: 2022-04-07 11:04:37
 */
#pragma once
#include "../include/utility.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
struct LidarEdegFactor
{

    LidarEdegFactor(Eigen::Vector3d currPoint, Eigen::Vector3d closestPoint1,
                    Eigen::Vector3d closestPoint2, double ratio) : currPoint_(currPoint),
                                                                   closestPoint1_(closestPoint1),
                                                                   closestPoint2_(closestPoint2),
                                                                   ratio_(ratio){
                                                                       //
                                                                   };

    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const
    {
        Eigen::Matrix<T, 3, 1> currP{T(currPoint_.x()), T(currPoint_.y()), T(currPoint_.z())};
        Eigen::Matrix<T, 3, 1> closeP1{T(closestPoint1_.x()), T(closestPoint1_.y()), T(closestPoint1_.z())};
        Eigen::Matrix<T, 3, 1> closeP2{T(closestPoint2_.x()), T(closestPoint2_.y()), T(closestPoint2_.z())};

        Eigen::Quaternion<T> Q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Quaternion<T> Q_Identity{T(1), T(0), T(0), T(0)};
        Q_last_curr = Q_Identity.slerp(T(ratio_), Q_last_curr);
        Eigen::Matrix<T, 3, 1> T_last_curr{t[0] * T(ratio_), t[1] * T(ratio_), t[2] * T(ratio_)};

        Eigen::Matrix<T, 3, 1> lastP = Q_last_curr * currP + T_last_curr;

        Eigen::Matrix<T, 3, 1> cross = (lastP - closeP1).cross(lastP - closeP2);
        Eigen::Matrix<T, 3, 1> det = closeP1 - closeP2;

        residual[0] = cross.x() / det.norm();
        residual[1] = cross.y() / det.norm();
        residual[2] = cross.z() / det.norm();
        return true;
    };

    static ceres::CostFunction *Create(const Eigen::Vector3d currPoint, const Eigen::Vector3d closestPoint1,
                                       const Eigen::Vector3d closestPoint2, const double ratio)
    {
        return (new ceres::AutoDiffCostFunction<LidarEdegFactor, 3, 4, 3>(
            new LidarEdegFactor(currPoint, closestPoint1, closestPoint2, ratio)));
    };

    Eigen::Vector3d currPoint_;
    Eigen::Vector3d closestPoint1_;
    Eigen::Vector3d closestPoint2_;
    double ratio_;
};

struct LidarSurfFactor
{
    LidarSurfFactor(Eigen::Vector3d currPoint, Eigen::Vector3d closestPoint,
                    Eigen::Vector3d closePoint1, Eigen::Vector3d closePoint2,
                    double ratio) : currPoint_(currPoint), closestPoint_(closestPoint),
                                    closePoint1_(closePoint1), closePoint2_(closePoint2),
                                    ratio_(ratio)
    {
        Eigen::Vector3d LinePP1 = closestPoint_ - closePoint1_;
        Eigen::Vector3d LinePP2 = closestPoint_ - closePoint2_;

        norm_ = LinePP1.cross(LinePP2);
        norm_.normalize();
    };

    template <typename T>
    bool operator()(const T *q, const T *t, T *residual) const
    {
        Eigen::Matrix<T, 3, 1> currP{T(currPoint_.x()), T(currPoint_.y()), T(currPoint_.z())};

        Eigen::Quaternion<T> Q_last_curr{q[3], q[0], q[1], q[2]};
        Eigen::Quaternion<T> Q_Identity{T(1), T(0), T(0), T(0)};

        Q_last_curr = Q_Identity.slerp(T(ratio_), Q_last_curr);
        Eigen::Matrix<T, 3, 1> T_last_curr{t[0] * T(ratio_), t[1] * T(ratio_), t[2] * T(ratio_)};

        Eigen::Matrix<T, 3, 1> lastP = Q_last_curr * currP + T_last_curr;
        Eigen::Matrix<T, 3, 1> closeP{T(closestPoint_.x()), T(closestPoint_.y()), T(closestPoint_.z())};
        Eigen::Matrix<T, 3, 1> norm{T(norm_.x()), T(norm_.y()), T(norm_.z())};
        residual[0] = (lastP - closeP).dot(norm);
        return true;
    };

    static ceres::CostFunction *Create(const Eigen::Vector3d currPoint, const Eigen::Vector3d closestPoint,
                                       const Eigen::Vector3d closePoint1, const Eigen::Vector3d closePoint2,
                                       const double ratio)
    {
        return (new ceres::AutoDiffCostFunction<LidarSurfFactor, 1, 4, 3>(
            new LidarSurfFactor(currPoint, closestPoint, closePoint1, closePoint2, ratio)));
    };

    Eigen::Vector3d currPoint_, closestPoint_, closePoint1_, closePoint2_, norm_;
    double ratio_;
};