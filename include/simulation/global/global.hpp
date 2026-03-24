#pragma once
#include <algorithm>
#include <Eigen/Dense>
#include <cmath>
#include <cppad/cppad.hpp>

namespace global {
    const extern double r_earth;
    const extern double gravity;
    const extern double pi;
    const extern double eps;
    const extern double dt;

    const extern Eigen::Matrix3d I3;
    const extern Eigen::Matrix4d I4;
    const extern Eigen::Matrix4d HI;
    const extern Eigen::Matrix3d Zero3x3;

    const extern Eigen::Vector3d Zero3;
    const extern Eigen::Vector3d i;
    const extern Eigen::Vector3d j;
    const extern Eigen::Vector3d k;

    double sec(double theta);
    double csc(double theta);
    double cot(double theta);


    double deg_to_rad(double deg);
    double rad_to_deg(double rad);

    double kts_to_ms(double kts);
    double ms_to_kts(double ms);
    double ft_to_m(double ft);
    double m_to_ft(double m);

    Eigen::Matrix3d hat(const Eigen::Vector3d& v);
    Eigen::Vector3d vee(const Eigen::Matrix3d& S);

    Eigen::Vector3d norm(const Eigen::Vector3d& v);
    double clamp_symmetric(double x, double max_abs);
    double clamp_positive(double x, double max_value);
    double clamp_to_1(double x);
    double wrap_to_pi(double x);

    double sin(double x);
    double cos(double x);
    double tan(double x);
    double asin(double x);
    double atan2(double y, double x);
    double sqrt(double x);
    double abs(double x);

    CppAD::AD<double> sin(const CppAD::AD<double>& x);
    CppAD::AD<double> cos(const CppAD::AD<double>& x);
    CppAD::AD<double> tan(const CppAD::AD<double>& x);
    CppAD::AD<double> asin(const CppAD::AD<double>& x);
    CppAD::AD<double> atan2(const CppAD::AD<double>& y, const CppAD::AD<double>& x);
    CppAD::AD<double> sqrt(const CppAD::AD<double>& x);
    CppAD::AD<double> abs(const CppAD::AD<double>& x);

    CppAD::AD<double> clamp_symmetric(const CppAD::AD<double>& x, double max_abs);
    CppAD::AD<double> clamp_positive(const CppAD::AD<double>& x, double max_value);
    CppAD::AD<double> clamp_to_1(const CppAD::AD<double>& x);

    double vector_norm(const Eigen::Vector3d& v);
    CppAD::AD<double> vector_norm(const Eigen::Matrix<CppAD::AD<double>, 3, 1>& v);

    Eigen::Matrix<CppAD::AD<double>, 3, 1> norm(const Eigen::Matrix<CppAD::AD<double>, 3, 1>& v);
}
