#pragma once
#include <Eigen/Dense>
#include "simulation/util/cppad.hpp"
#include "simulation/util/linalg.hpp"
#include "simulation/util/trig.hpp"
#include "simulation/util/units.hpp"

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

    double first_order_lag(double val, double prev_val, double tau);
    Eigen::Vector3d first_order_lag(const Eigen::Vector3d& val, const Eigen::Vector3d& prev_val, double tau);
    Eigen::Quaterniond first_order_lag(const Eigen::Quaterniond& val, const Eigen::Quaterniond& prev_val, double tau);

}
