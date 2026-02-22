#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace global {
    const extern double r_earth;
    const extern double gravity;
    const extern double pi;
    const extern double eps;

    const extern Eigen::Matrix3d I3;
    const extern Eigen::Matrix4d I4;
    const extern Eigen::Matrix4d HI;


    double sec(double theta);
    double csc(double theta);
    double cot(double theta);


    double deg2rad(double deg);
    double rad2deg(double rad);


    Eigen::Matrix3d hat(const Eigen::Vector3d& v);
    Eigen::Vector3d vee(const Eigen::Matrix3d& S);


}