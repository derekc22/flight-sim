#include <optional>
#include <random>
#include "simulation/constants/public/scalars.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/sensors/public/components/base.hpp"
#include "simulation/util/public/filters.hpp"

namespace sensors {

    Sensor::Sensor(double mean, double stddev, double bias, const Eigen::Vector3d& bias3d, double tau) : mean(mean), stddev(stddev), bias(bias), bias_3d(bias3d), dist(mean, stddev), tau(tau) {}

    double Sensor::step(double meas, std::optional<double>& lag_state, double dt) {
        double prev_meas = lag_state ? lag_state.value() : meas;
        double meas_lagged = util::first_order_lag(meas, prev_meas, tau, dt); // apply EMA
        lag_state = meas_lagged;
        double meas_biased = meas_lagged + bias; // apply bias
        double noise = dist(gen);
        double meas_noised = meas_biased + noise; // apply Gaussian noise
        return meas_noised;
    }

    Eigen::Vector3d Sensor::step(const Eigen::Vector3d& meas, std::optional<Eigen::Vector3d>& lag_state, double dt) {
        Eigen::Vector3d prev_meas = lag_state ? lag_state.value() : meas;
        Eigen::Vector3d meas_lagged = util::first_order_lag(meas, prev_meas, tau, dt); // apply EMA
        lag_state = meas_lagged;
        Eigen::Vector3d meas_biased = meas_lagged + bias_3d; // apply bias
        Eigen::Vector3d noise(dist(gen), dist(gen), dist(gen));
        Eigen::Vector3d meas_noised = meas_biased + noise; // apply Gaussian noise
        return meas_noised;
    }

    Eigen::Quaterniond Sensor::step(const Eigen::Quaterniond& meas, std::optional<Eigen::Quaterniond>& lag_state, double dt) {
        Eigen::Quaterniond prev_meas = lag_state ? lag_state.value() : meas;
        Eigen::Quaterniond meas_lagged = util::first_order_lag(meas, prev_meas, tau, dt); // apply EMA via quaternion SLERP
        lag_state = meas_lagged;

        double bias_angle = bias_3d.norm();
        Eigen::Quaterniond bias_q;
        if (bias_angle < constants::eps) bias_q = constants::qI;
        else {
            Eigen::Vector3d bias_axis = bias_3d / bias_angle;
            bias_q = Eigen::Quaterniond(Eigen::AngleAxisd(bias_angle, bias_axis));
        }
        Eigen::Quaterniond meas_biased = meas_lagged * bias_q; // apply bias

        Eigen::Vector3d noise_dtheta(dist(gen), dist(gen), dist(gen));
        double angle = noise_dtheta.norm();

        Eigen::Quaterniond dq;
        if (angle < constants::eps) dq = constants::qI;
        else {
            Eigen::Vector3d axis = noise_dtheta / angle;
            dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
        }
        Eigen::Quaterniond meas_noised = meas_biased * dq; // apply Gaussian noise

        meas_noised.normalize();
        return meas_noised;
    }

}
