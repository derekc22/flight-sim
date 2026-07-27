#pragma once
#include <array>
#include <cstddef>
#include <Eigen/Dense>

namespace util {

    double clamp(double x, double min_value, double max_value);
    double clamp_symmetric(double x, double max_abs);
    double clamp_positive(double x, double max_value);
    double clamp_to_1(double x);
    double clamp_inside_1(double x);
    double wrap_to_pi(double x);

    double sqrt(double x);
    double abs(double x);
    double smooth_abs(double x);

    double first_order_lag(double val, double prev_val, double tau, double dt);
    Eigen::Vector3d first_order_lag(const Eigen::Vector3d& val, const Eigen::Vector3d& prev_val, double tau, double dt);
    Eigen::Quaterniond first_order_lag(const Eigen::Quaterniond& val, const Eigen::Quaterniond& prev_val, double tau, double dt);

    Eigen::VectorXd vec_clamp(const Eigen::VectorXd& x, const Eigen::VectorXd& x_min, const Eigen::VectorXd& x_max);
    bool vec_is_close(const Eigen::VectorXd& a, const Eigen::VectorXd& b);

    template <typename T, std::size_t N>
    void fill_arr(std::array<T, N>& x, std::size_t start, std::size_t stop, const T& val);
}

#include "simulation/util/public.tpp"
