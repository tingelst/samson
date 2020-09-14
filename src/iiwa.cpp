#include <iostream>
using namespace std;

#include <samson/so3.hpp>
#include <samson/se3.hpp>
#include <samson/types.hpp>
#include <samson/robot.hpp>

constexpr double pi = 3.14159265358979323846;

int main()
{

  dual L1 = 0.36;
  dual L2 = 0.42;
  dual L3 = 0.4;
  dual L4 = 0.126;

  Matrix4dual M;
  M << 1.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 1.0, L1 + L2 + L3 + L4,
       0.0, 0.0, 0.0, 1.0;

  MatrixXdual S(7, 6);
  S << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, L1, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, L1+L2, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, L1+L2+L3, 0.0,
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
  S.transposeInPlace();

  std::cout << S.cols() << std::endl;

  VectorXdual angles(7);
  angles << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;

  std::cout << samson::robot::fk(M, S, angles) << std::endl;
}