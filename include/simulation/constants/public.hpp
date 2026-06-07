#pragma once
#include <cstddef>
#include <Eigen/Dense>

namespace constants {

    // Constants
    inline constexpr double r_earth = 6.371e6;      // [m]
    inline constexpr double g_earth = 9.80665;      // [ms^-2]
    inline constexpr double pi = 3.14159265358979;
    inline constexpr double eps = 1e-9;
    inline constexpr double eps_strict = 1e-12;
    inline constexpr double dt = 0.01;              // [s]
    inline constexpr double gamma_air = 1.4;        // [-]
    inline constexpr double R_air = 287.052874247;  // [J·kg^-1·K^-1]
    inline constexpr double lapse_rate = -0.0065;   // [K/m]
    inline constexpr double P_SL = 101325;          // [Pa]
    inline constexpr double rho_SL = 1.225;         // [kg/m^3]
    inline constexpr double T_SL = 288.0;           // [K]

    // Special matrices
    inline const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    inline const Eigen::Matrix4d HI = Eigen::Matrix4d::Identity(); // Identity homogeneous transformation matrix
    inline const Eigen::Matrix3d Zero3x3 = Eigen::Matrix3d::Zero();

    // Special vectors
    inline const Eigen::Vector3d Zero3 = Eigen::Vector3d::Zero();
    inline const Eigen::Vector3d ei = Eigen::Vector3d(1, 0, 0);
    inline const Eigen::Vector3d ej = Eigen::Vector3d(0, 1, 0);
    inline const Eigen::Vector3d ek = Eigen::Vector3d(0, 0, 1);

    // Special quaternions
    inline const Eigen::Quaterniond qI = Eigen::Quaterniond::Identity();


    // Template alias vectors
    template <typename T>
    using Vector3_T = Eigen::Matrix<T, 3, 1>;

    template <typename T>
    inline const Vector3_T<T> Zero3_T = Vector3_T<T>::Zero();

    template <typename T>
    using Matrix3_T = Eigen::Matrix<T, 3, 3>;


    // Selected state and input dimension
    inline constexpr std::size_t state_dim = 8; 

    inline constexpr std::size_t surface_input_dim = 3;
    inline constexpr std::size_t fixed_surface_input_dim = 2;

    inline constexpr std::size_t propulsor_input_dim = 3;
    inline constexpr std::size_t fixed_propulsor_input_dim = 0;

    inline constexpr std::size_t full_surface_input_dim = surface_input_dim + fixed_surface_input_dim;
    inline constexpr std::size_t full_propulsor_input_dim = propulsor_input_dim + fixed_propulsor_input_dim;

    inline constexpr std::size_t fixed_input_dim = fixed_surface_input_dim + fixed_propulsor_input_dim;
    inline constexpr std::size_t input_dim = surface_input_dim + propulsor_input_dim;
    inline constexpr std::size_t full_input_dim = input_dim + fixed_input_dim;

    inline constexpr std::size_t state_input_dim = state_dim + input_dim;
}
