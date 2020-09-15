#include <iostream>
#include <fstream>
using namespace std;

#include <samson/so3.hpp>
#include <samson/se3.hpp>
#include <samson/types.hpp>
#include <samson/robot.hpp>

constexpr double pi = 3.14159265358979323846;

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/SVD>
using namespace Eigen;

auto create_affine_matrix(dual a, dual b, dual c, Vector3dual trans) -> Matrix4dual
{
    Transform<dual, 3, Eigen::Affine> t;
    t = Translation<dual, 3>(trans);
    t.rotate(AngleAxis<dual>(a, Vector3dual::UnitX()));
    t.rotate(AngleAxis<dual>(b, Vector3dual::UnitY()));
    t.rotate(AngleAxis<dual>(c, Vector3dual::UnitZ()));
    return t.matrix();
}

auto pos(const Matrix4dual &X) -> Vector3dual
{
    return X.block<3, 1>(0, 3);
}

auto pinv(const MatrixXd &a, double epsilon = std::numeric_limits<double>::epsilon()) -> MatrixXd
{
    JacobiSVD<MatrixXd> svd(a, ComputeThinU | ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
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
        1.0, 0.0, 0.0, 0.0, L1 + L2, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 0.0, L1 + L2 + L3, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    //   S.transposeInPlace();

    //   std::cout << S << std::endl;

    VectorXdual angles(7);
    angles << -1.0, -1.75, 0.0, 0.03, 0.0, 0.67, 0.0;

    std::cout << samson::robot::fk(M, S, angles) << "\n"
              << std::endl;

    Vector3dual target_position;
    target_position << -0.3, 0.4, 0.5;

    auto fk = [&](const VectorXdual &angles) -> Matrix4dual {
        return samson::robot::fk(M, S, angles);
    };

    auto task = [&](const VectorXdual &angles) -> Vector3dual {
        return target_position - pos(fk(angles));
    };

    auto task2 = [&](const VectorXdual &target, const VectorXdual &angles) -> VectorXdual {
        return target - angles;
    };

    MatrixXd traj(7, 100000);

    VectorXdual res1;
    VectorXdual res2;
    Eigen::MatrixXd Jac1;
    MatrixXd Jac2;

    using Matrix7d = Matrix<double, 7, 7>;
    Matrix7d I7 = Matrix7d::Identity();

    double dt = 0.01;

    int k = 0;
    for (k; k < 100000; ++k)
    {
        VectorXd target_angles(7);
        target_angles << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;

        Jac1 = jacobian(task, wrt(angles), at(angles), res1);
        Jac2 = jacobian(task2, wrt(angles), at(target_angles, angles), res2);

        MatrixXd Jac1pinv = pinv(Jac1);
        MatrixXd Jac2pinv = pinv(Jac2);

        VectorXd dangles = Jac1pinv * (-res1.cast<double>() * 0.01) + (I7 - Jac1pinv * Jac1) * Jac2pinv * (-res2.cast<double>() * 0.01);

        angles = angles + dangles * 0.01;

        traj.block<7, 1>(0, k) = angles.cast<double>();
    }

    std::cout << angles << "\n"
              << std::endl;
    std::cout << fk(angles) << std::endl;

    std::ofstream fout("traj.txt");
    if (fout)
    {
        fout << traj.transpose();
    }
}