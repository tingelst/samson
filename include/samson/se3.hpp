#pragma once

#include <Eigen/Core>
using namespace Eigen;

#include <autodiff/forward.hpp>
#include <autodiff/forward/eigen.hpp>
using namespace autodiff;

#include <samson/so3.hpp>

namespace samson::se3
{

    using Vector6dual = Matrix<dual, 6, 1, 0, 6, 1>;
    using Matrix4dual = Matrix<dual, 4, 4, 0, 4, 4>;

    auto se3(const Vector6dual &v) -> Matrix4dual
    {
        Matrix4dual X;
        X << 0.0, -v(2), v(1), v(3),
            v(2), 0.0, -v(0), v(4),
            -v(1), v(0), 0.0, v(5),
            0.0, 0.0, 0.0, 0.0;
        return X;
    }

    auto se3(const Matrix4dual &X) -> Vector6dual
    {
        Vector6dual v;
        v << X(2, 1), X(0, 2), X(1, 0), X(0, 3), X(1, 3), X(2, 3);
        return v;
    }

    auto exp(Vector6dual &x) -> Matrix4dual
    {
        using samson::so3::from_axis_angle;
        using samson::so3::so3;

        Vector3dual w = x.head<3>();
        Vector3dual v = x.tail<3>();

        dual theta = w.norm();
        Matrix4dual ret = Matrix4dual::Identity();
        if (theta == 0.0)
        {
            ret = Matrix4dual::Identity();
            ret.block<3, 1>(0, 3) = v;
            return ret;
        }

        Vector3dual k = w.normalized();
        dual sin_theta = sin(theta);
        dual one_minus_cos_theta = 1.0 - cos(theta);
        Matrix3dual K = so3(k);
        Matrix3dual KK = K * K;

        Matrix3dual I = Matrix3dual::Identity();

        ret.block<3, 3>(0, 0) = from_axis_angle(k, theta);
        ret.block<3, 1>(0, 3) = (I * theta + one_minus_cos_theta * K + (theta - sin_theta) * KK) * (v / theta);

        return ret;
    }

    auto exp(Matrix4dual &X) -> Matrix4dual
    {
        Vector6dual x = se3(X);
        return exp(x);
    }
} // namespace samson::se3