#pragma once
#include <Eigen/Dense>
#include <optional>
#include <random>
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"

namespace sensors {

    // directly measured
    struct AngleOfAttackMeasurement : aerodynamics::AngleOfAttack {};

    struct TranslationalAccelerationMeasurement : dynamics::TranslationalAcceleration {};

    struct AngularVelocityMeasurement : dynamics::AngularVelocity {};

    struct StagnationAirPressureMeasurement : atmospheric::StagnationAirPressure {};

    struct StaticAirPressureMeasurement : atmospheric::StaticAirPressure {};

    struct StagnationAirTemperatureMeasurement : atmospheric::StagnationAirTemperature {};

    struct PositionMeasurement : dynamics::Position {};

    struct TranslationalVelocityMeasurement : dynamics::TranslationalVelocity {};

    struct HeadingMeasurement : geography::Heading {};




    /** @note For simplicity, the internal dynamics of some sensors will remain unmodelled 
        Instead, these sensors will simply output a noised, lagged version of the corresponding ground-truth quantity
    */
    struct Sensor {
        double mean;
        double stddev;
        double bias;
        Eigen::Vector3d bias_3d;
        std::mt19937 gen{std::random_device{}()};
        std::normal_distribution<double> dist;
        double tau;

        Sensor(double mean, double stddev, double bias, const Eigen::Vector3d& bias3d, double tau);

        double step(double meas, std::optional<double>& lag_state);
        Eigen::Vector3d step(const Eigen::Vector3d& meas, std::optional<Eigen::Vector3d>& lag_state);
        Eigen::Quaterniond step(const Eigen::Quaterniond& meas, std::optional<Eigen::Quaterniond>& lag_state);
    };

    struct AngleOfAttackVane : Sensor {
        std::optional<double> prev_alpha_lag;
        AngleOfAttackMeasurement measure(const aerodynamics::AngleOfAttack& alpha);
    };

    struct Accelerometer : Sensor {
        std::optional<Eigen::Vector3d> prev_fB_lag;
        TranslationalAccelerationMeasurement measure(const dynamics::TranslationalAcceleration& fB);
        // Note: fB = aB_BI - gB 
        //          = FB_net/m - gB
        // Where 
        // fB is specific force 
        // aB_BI is inertial acceleration (the inerital derivative of velocity - as opposed to the body derivative of velocity, vB_BI_dot)
        // That is, an accelerometer measures specific force - which is inertial acceleration minus the contribution from gravity and expressed in the body frame
    };

    struct Gyroscope : Sensor {
        std::optional<Eigen::Vector3d> prev_wB_BI_lag;
        AngularVelocityMeasurement measure(const dynamics::AngularVelocity& wB_BI);
    };

    struct PitotTube : Sensor {
        std::optional<double> prev_P0_lag;
        StagnationAirPressureMeasurement measure(const atmospheric::StagnationAirPressure& P0);
    };

    struct StaticPort : Sensor {
        std::optional<double> prev_P_lag;
        StaticAirPressureMeasurement measure(const atmospheric::StaticAirPressure& P);
    };

    struct TotalAirTemperatureProbe : Sensor {
        std::optional<double> prev_T0_lag;
        StagnationAirTemperatureMeasurement measure(const atmospheric::StagnationAirTemperature& T0);
    };

    struct GNSSReceiver : Sensor {
        std::optional<Eigen::Vector3d> prev_pI_BI_lag;
        std::optional<Eigen::Vector3d> prev_vB_BI_lag;
        PositionMeasurement measure(const dynamics::Position& pI_BI);
        TranslationalVelocityMeasurement measure(const dynamics::TranslationalVelocity& vB_BI);
    };

    struct Magnetometer : Sensor {
        std::optional<double> prev_heading_lag;
        HeadingMeasurement measure(const geography::Heading& heading);
    };

}
