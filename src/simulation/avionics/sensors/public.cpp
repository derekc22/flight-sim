#include <optional>
#include <random>
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/avionics/sensors/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/util/public.hpp"

namespace avionics {

    Sensor::Sensor(double mean, double stddev, double bias, const Eigen::Vector3d& bias3d, double tau) : mean(mean), stddev(stddev), bias(bias), bias_3d(bias3d), dist(mean, stddev), tau(tau) {}

    double Sensor::step(double meas, std::optional<double>& lag_state) {
        double prev_meas = lag_state ? lag_state.value() : meas;
        double meas_lagged = util::first_order_lag(meas, prev_meas, tau); // apply EMA
        lag_state = meas_lagged;
        double meas_biased = meas_lagged + bias; // apply bias
        double noise = dist(gen);
        double meas_noised = meas_biased + noise; // apply Gaussian noise
        return meas_noised;
    }

    Eigen::Vector3d Sensor::step(const Eigen::Vector3d& meas, std::optional<Eigen::Vector3d>& lag_state) {
        Eigen::Vector3d prev_meas = lag_state ? lag_state.value() : meas;
        Eigen::Vector3d meas_lagged = util::first_order_lag(meas, prev_meas, tau); // apply EMA
        lag_state = meas_lagged;
        Eigen::Vector3d meas_biased = meas_lagged + bias_3d; // apply bias
        Eigen::Vector3d noise(dist(gen), dist(gen), dist(gen));
        Eigen::Vector3d meas_noised = meas_biased + noise; // apply Gaussian noise
        return meas_noised;
    }

    Eigen::Quaterniond Sensor::step(const Eigen::Quaterniond& meas, std::optional<Eigen::Quaterniond>& lag_state) {
        Eigen::Quaterniond prev_meas = lag_state ? lag_state.value() : meas;
        Eigen::Quaterniond meas_lagged = util::first_order_lag(meas, prev_meas, tau); // apply EMA via quaternion SLERP
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

    AngleOfAttackMeasurement AngleOfAttackVane::measure(const aerodynamics::AngleOfAttack& alpha) {
        return { step(alpha.data, prev_alpha_lag) };
    }

    TranslationalAccelerationMeasurement Accelerometer::measure(const dynamics::TranslationalAcceleration& fB) {
        return { step(fB.data, prev_fB_lag) };
    }

    AngularVelocityMeasurement Gyroscope::measure(const dynamics::AngularVelocity& wB_BI) {
        return { step(wB_BI.data, prev_wB_BI_lag) };
    }

    StagnationAirPressureMeasurement PitotTube::measure(const atmospheric::StagnationAirPressure& P0) {
        return { step(P0.data, prev_P0_lag) };
    }

    StaticAirPressureMeasurement StaticPort::measure(const atmospheric::StaticAirPressure& P) {
        return { step(P.data, prev_P_lag) };
    }

    StagnationAirTemperatureMeasurement TotalAirTemperatureProbe::measure(const atmospheric::StagnationAirTemperature& T0) {
        return { step(T0.data, prev_T0_lag) };
    }

    PositionMeasurement GNSSReceiver::measure(const dynamics::Position& pI_BI) {
        return { step(pI_BI.data, prev_pI_BI_lag) };
    }

    TranslationalVelocityMeasurement GNSSReceiver::measure(const dynamics::TranslationalVelocity& vB_BI) {
        return { step(vB_BI.data, prev_vB_BI_lag) };
    }
    
    HeadingMeasurement Magnetometer::measure(const geography::Heading& heading) {
        return { step(heading.data, prev_heading_lag) };
    }

}
