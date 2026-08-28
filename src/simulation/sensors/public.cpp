#include <optional>
#include <random>
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/sensors/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/util/public.hpp"

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

    AngleOfAttackMeasurement AngleOfAttackVane::measure(const aerodynamics::AngleOfAttack& alpha, double dt) {
        return { step(alpha.data, prev_alpha_lag, dt) };
    }

    TranslationalAccelerationMeasurement Accelerometer::measure(const dynamics::TranslationalAcceleration& fB, double dt) {
        return { step(fB.data, prev_fB_lag, dt) };
    }

    AngularVelocityMeasurement Gyroscope::measure(const dynamics::AngularVelocity& wB_BI, double dt) {
        return { step(wB_BI.data, prev_wB_BI_lag, dt) };
    }

    StagnationAirPressureMeasurement PitotTube::measure(const atmospheric::StagnationAirPressure& P0, double dt) {
        return { step(P0.data, prev_P0_lag, dt) };
    }

    StaticAirPressureMeasurement StaticPort::measure(const atmospheric::StaticAirPressure& P, double dt) {
        return { step(P.data, prev_P_lag, dt) };
    }

    StagnationAirTemperatureMeasurement TotalAirTemperatureProbe::measure(const atmospheric::StagnationAirTemperature& T0, double dt) {
        return { step(T0.data, prev_T0_lag, dt) };
    }

    PositionMeasurement GNSSReceiver::measure(const dynamics::Position& pI_BI, double dt) {
        return { step(pI_BI.data, prev_pI_BI_lag, dt) };
    }

    TranslationalVelocityMeasurement GNSSReceiver::measure(const dynamics::TranslationalVelocity& vB_BI, double dt) {
        return { step(vB_BI.data, prev_vB_BI_lag, dt) };
    }
    
    HeadingMeasurement Magnetometer::measure(const geography::Heading& heading, double dt) {
        return { step(heading.data, prev_heading_lag, dt) };
    }

    SensorMeasurements SensorManager::step(const SensorGroundTruth& sensor_gt, double dt) {
        SensorMeasurements sensor_meas {
            .alpha = sensors.aoa_vane.measure(sensor_gt.alpha, dt),
            .fB = sensors.accelerometer.measure(sensor_gt.fB, dt),
            .wB_BI = sensors.gyro.measure(sensor_gt.wB_BI, dt),
            .P0 = sensors.pitot_tube.measure(sensor_gt.P0, dt),
            .P = sensors.static_port.measure(sensor_gt.P, dt),
            .T0 = sensors.tat_probe.measure(sensor_gt.T0, dt),
            .pI_BI_gnss = sensors.gnss.measure(sensor_gt.pI_BI, dt),
            .vB_BI_gnss = sensors.gnss.measure(sensor_gt.vB_BI, dt),
            .heading_BE = sensors.magnetometer.measure(sensor_gt.heading, dt)
        };

        hist = sensor_meas;

        return sensor_meas;
    }

    SensorGroundTruth build_sensor_gt(
        const dynamics::RigidBodyState& Xt,
        const dynamics::RigidBodyState& XEt,
        const aerodynamics::AerodynamicState& aero_t,
        const atmospheric::StaticAtmosphericState& atm_t,
        const dynamics::Mass& mass,
        const dynamics::Wrench& WB_net
    ) {

        atmospheric::MachNumber Mach = atmospheric::mps_to_mach(Xt.v, atm_t.T);
        atmospheric::StagnationAtmosphericState atm0_t = atmospheric::static_to_stagnation(atm_t, Mach);
        dynamics::Gravity gB = geography::gB(XEt.p, XEt.q);
        dynamics::EulerAngles eul;
        eul.set(Xt.q);

        SensorGroundTruth sensor_gt = {
            .alpha = aero_t.alpha,
            .fB = dynamics::TranslationalAcceleration{ WB_net.F.data / mass.data - gB.data },
            .wB_BI = Xt.w,
            .P0 = atm0_t.P0,
            .P = atm_t.P,
            .T0 = atm0_t.T0,
            .pI_BI = Xt.p,
            .vB_BI = Xt.v,
            .heading = geography::Heading{ eul.psi() },
        };

        return sensor_gt;
    }

}
