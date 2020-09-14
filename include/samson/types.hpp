#pragma once

#include <Eigen/Core>
#include <autodiff/forward/eigen.hpp>

namespace samson::types
{
    using autodiff::Vector3dual;
    using Vector6dual = Eigen::Matrix<dual, 6, 1>;
    using Matrix6dual = Eigen::Matrix<dual, 6, 6>;

} // namespace samson::types
