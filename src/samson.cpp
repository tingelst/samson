#include <iostream>
using namespace std;

#include <samson/so3.hpp>
#include <samson/se3.hpp>
#include <samson/types.hpp>
#include <samson/robot.hpp>

constexpr double pi = 3.14159265358979323846;

using Axis = samson::types::Vector3dual;
using Angle = dual;

using Twist = samson::types::Vector6dual;

int main()
{

  Axis v;
  v << 0.0, 0.0, 1.0;

  Angle th{pi / 3};

  std::cout << samson::so3::from_axis_angle(v, th) << std::endl;

  Twist x;
  x << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  // std::cout << samson::se3::exp(x) << std::endl;

  struct Robot
  {
    Matrix4dual M;
    M << -1.0, 0.0, 0.0, L1 + L2,
        0.0, 0.0, 1.0, W1 + W2,
        0.0, 1.0, 0.0, H1 - H2,
        0.0, 0.0, 0.0, 1.0;
  };

  dual W1 = 0.10915;
  dual W2 = 0.0823;
  dual L1 = 0.425;
  dual L2 = 0.39225;
  dual H1 = 0.089159;
  dual H2 = 0.09465;

  Matrix4dual M;
  M << -1.0, 0.0, 0.0, L1 + L2,
      0.0, 0.0, 1.0, W1 + W2,
      0.0, 1.0, 0.0, H1 - H2,
      0.0, 0.0, 0.0, 1.0;

  MatrixXdual S(6, 6);
  S << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, -H1, 0.0, 0.0,
      0.0, 1.0, 0.0, -H1, 0.0, L1,
      0.0, 1.0, 0.0, -H1, 0.0, L1 + L2,
      0.0, 0.0, -1.0, -W1, L1 + L2, 0.0,
      0.0, 1.0, 0.0, H2 - H1, 0.0, L1 + L2;
  S.transposeInPlace();

  VectorXdual angles(6);
  angles << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  std::cout << samson::robot::fk(M, S, angles) << std::endl;
}