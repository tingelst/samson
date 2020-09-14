#include <iostream>
using namespace std;

#include <samson/so3.hpp>
#include <samson/se3.hpp>
#include <samson/types.hpp>
#include <samson/robot.hpp>

constexpr double pi = 3.14159265358979323846;

#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

Matrix4dual create_affine_matrix(dual a, dual b, dual c, Vector3dual trans)
{
    Transform<dual, 3, Eigen::Affine> t;
    t = Translation<dual, 3>(trans);
    t.rotate(AngleAxis<dual>(a, Vector3dual::UnitX()));
    t.rotate(AngleAxis<dual>(b, Vector3dual::UnitY()));
    t.rotate(AngleAxis<dual>(c, Vector3dual::UnitZ()));
    return t.matrix();
}

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
      0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

//   S.transposeInPlace();

//   std::cout << S << std::endl;

  VectorXdual angles(7);
  angles << pi/3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  std::cout << samson::robot::fk(M, S, angles) <<  "\n" << std::endl;

  Vector3dual t;
  t << 1,2,3;
  std::cout << create_affine_matrix(pi/3, 0.0, 0.0, t ) << std::endl;
}