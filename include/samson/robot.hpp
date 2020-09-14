#pragma once

#include <Eigen/Core>
using namespace Eigen;

#include <autodiff/forward.hpp>
#include <autodiff/forward/eigen.hpp>
using namespace autodiff;

#include <samson/so3.hpp>

namespace samson::robot
{
    using namespace samson::se3;

    auto fk(const Matrix4dual &M, const MatrixXdual &S, const VectorXdual &theta)
    {
        Matrix4dual T = M;
        for (int i = 0; i < S.rows(); ++i)
        {
            Vector6dual Vi = S.row(i);
            dual thetai = theta(i);
            Vi *= thetai;
            T *= exp(Vi);
        }
        return T;
    }

} // namespace samson::robot