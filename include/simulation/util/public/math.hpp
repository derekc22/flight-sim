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

    Eigen::VectorXd vec_clamp(const Eigen::VectorXd& x, const Eigen::VectorXd& x_min, const Eigen::VectorXd& x_max);
    bool vec_is_close(const Eigen::VectorXd& a, const Eigen::VectorXd& b);

    template <typename T, std::size_t N>
    void fill_arr(std::array<T, N>& x, std::size_t start, std::size_t stop, const T& val);
}

#include "simulation/util/public/math.tpp"
