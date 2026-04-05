#pragma once
#include <Eigen/Dense>

namespace constants {

    // Constants
    inline constexpr double r_earth = 6.371e6;      // [m]
    inline constexpr double g_earth = 9.80665;      // [ms^-2]
    inline constexpr double pi = 3.1415926535897932384626433832795028841971693993751;
    inline constexpr double eps = 1e-9;
    inline constexpr double dt = 0.001;             // [s]
    inline constexpr double gamma_air = 1.4;        // [-]
    inline constexpr double R_air = 287.052874247;  // [J·kg^-1·K^-1]
    inline constexpr double lapse_rate = -0.0065;   // [K/m]
    inline constexpr double P_SL = 101325;         // [Pa]
    inline constexpr double rho_SL = 1.225;         // [kg/m^3]
    inline constexpr double T_SL = 288.0;           // [K]

    // Special Matrices
    inline const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    inline const Eigen::Matrix4d HI = Eigen::Matrix4d::Identity(); // Identity homogenous transformation matrix
    inline const Eigen::Matrix3d Zero3x3 = Eigen::Matrix3d::Zero();

    // Special Vectors
    inline const Eigen::Vector3d Zero3 = Eigen::Vector3d::Zero();
    inline const Eigen::Vector3d ei = Eigen::Vector3d(1, 0, 0);
    inline const Eigen::Vector3d ej = Eigen::Vector3d(0, 1, 0);
    inline const Eigen::Vector3d ek = Eigen::Vector3d(0, 0, 1);

}