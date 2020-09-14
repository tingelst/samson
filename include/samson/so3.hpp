#pragma once

#include <Eigen/Core>
using namespace Eigen;

#include <autodiff/forward.hpp>
#include <autodiff/forward/eigen.hpp>
using namespace autodiff;

namespace samson::so3
{
    auto so3(const Vector3dual &v) -> Matrix3dual
    {
        Matrix3dual X;
        X << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
        return X;
    }

    auto so3(const Matrix3dual &X) -> Vector3dual
    {
        Vector3dual v;
        v << X(2, 1), X(0, 2), X(1, 0);
        return v;
    }

    auto to_axis_angle(const Vector3dual &v) -> std::pair<Vector3dual, dual>
    {
        dual theta = v.norm();
        Vector3dual k = v.normalized();
        return std::make_pair(k, theta);
    }

    auto exp(const Vector3dual &v) -> Matrix3dual
    {
        auto [k, th] = to_axis_angle(v);
        Matrix3dual K = so3(k);
        return Matrix3dual::Identity() + sin(th) * K + (1.0 - cos(th)) * K * K;
    }

    auto exp(const Matrix3dual &X) -> Matrix3dual
    {
        Vector3dual v = so3(X);
        return exp(v);
    }

    auto from_axis_angle(const Vector3dual &axis, dual angle)
    {
        Vector3dual x = axis * angle;
        Matrix3dual X = so3(x);
        return exp(X);
    }

} // namespace samson::so3