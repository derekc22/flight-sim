#pragma once
#include <Eigen/Dense>
#include <optional>
#include <random>
#include "simulation/aerodynamics/shared.hpp"
#include "simulation/atmospheric/shared.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/geography/shared.hpp"

namespace avionics {

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


    struct SensorMeasurements {
        AngleOfAttackMeasurement alpha;
        TranslationalAccelerationMeasurement accel;
        AngularVelocityMeasurement wB_BI;
        StagnationAirPressureMeasurement P0;
        StaticAirPressureMeasurement P;
        StagnationAirTemperatureMeasurement T0;
        PositionMeasurement pI_BI_gnss;
        TranslationalVelocityMeasurement vB_BI_gnss;
        HeadingMeasurement heading_BE;
    };

    // Note: Due to modelling complexity, the internal dynamics of some sensors will remain unmodelled
    // Instead, these sensor will simply output a noised, delayed version of the corresponding ground-truth quantity

    struct Sensor {
        double mean;
        double stddev;
        double bias;
        Eigen::Vector3d bias_3d;
        std::mt19937 gen{std::random_device{}()};
        std::normal_distribution<double> dist;
        double tau;

        Sensor(double mean, double stddev, double bias, const Eigen::Vector3d& bias3d, double tau);

        double _step(double meas, std::optional<double>& lag_state);
        Eigen::Vector3d _step(const Eigen::Vector3d& meas, std::optional<Eigen::Vector3d>& lag_state);
        Eigen::Quaterniond _step(const Eigen::Quaterniond& meas, std::optional<Eigen::Quaterniond>& lag_state);
    };

    struct AngleOfAttackVane : Sensor {
        std::optional<double> prev_alpha_lag;
        AngleOfAttackMeasurement _measure(const aerodynamics::AngleOfAttack& alpha);
    };

    struct Accelerometer : Sensor {
        std::optional<Eigen::Vector3d> prev_accel_lag;
        TranslationalAccelerationMeasurement _measure(const dynamics::TranslationalAcceleration& accelB);
        // Note: accelB = FB_net/m - gB, not vB_BI_dot
        // That is, an accelerometer measures all accelerations excluding gravity 
    };

    struct Gyroscope : Sensor {
        std::optional<Eigen::Vector3d> prev_wB_BI_lag;
        AngularVelocityMeasurement _measure(const dynamics::AngularVelocity& wB_BI);
    };

    struct PitotTube : Sensor {
        std::optional<double> prev_P0_lag;
        StagnationAirPressureMeasurement _measure(const atmospheric::StagnationAirPressure& P0);
    };

    struct StaticPort : Sensor {
        std::optional<double> prev_P_lag;
        StaticAirPressureMeasurement _measure(const atmospheric::StaticAirPressure& P);
    };

    struct TotalAirTemperatureProbe : Sensor {
        std::optional<double> prev_T0_lag;
        StagnationAirTemperatureMeasurement _measure(const atmospheric::StagnationAirTemperature& T0);
    };

    struct GNSSReceiver : Sensor {
        std::optional<Eigen::Vector3d> prev_pI_BI_lag;
        std::optional<Eigen::Vector3d> prev_vB_BI_lag;
        PositionMeasurement _measure(const dynamics::Position& pI_BI);
        TranslationalVelocityMeasurement _measure(const dynamics::TranslationalVelocity& vB_BI);
    };

    struct Magnetometer : Sensor {
        std::optional<double> prev_heading_lag;
        HeadingMeasurement _measure(const geography::Heading& heading);
    };

}
