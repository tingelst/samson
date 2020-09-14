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
        for (int i = 0; i < S.cols(); ++i)
        {
            Vector6dual Vi = S.block<6, 1>(0, i);
            std::cout << Vi << "\n" << std::endl;
            Vi *= theta(i);
            Matrix4dual Ti = exp(Vi);
            std::cout << Ti << "\n" << std::endl;
            T *= Ti;
        }
        return T;
    }

} // namespace samson::robot