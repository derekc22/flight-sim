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

        double step(double meas, std::optional<double>& lag_state, double dt);
        Eigen::Vector3d step(const Eigen::Vector3d& meas, std::optional<Eigen::Vector3d>& lag_state, double dt);
        Eigen::Quaterniond step(const Eigen::Quaterniond& meas, std::optional<Eigen::Quaterniond>& lag_state, double dt);
    };

    struct AngleOfAttackVane : Sensor {
        std::optional<double> prev_alpha_lag;
        AngleOfAttackMeasurement measure(const aerodynamics::AngleOfAttack& alpha, double dt);
    };

    struct Accelerometer : Sensor {
        std::optional<Eigen::Vector3d> prev_fB_lag;
        TranslationalAccelerationMeasurement measure(const dynamics::TranslationalAcceleration& fB, double dt);
        // Note: fB = aB_BI - gB 
        //          = FB_net/m - gB
        // Where 
        // fB is specific force 
        // aB_BI is inertial acceleration (the inerital derivative of velocity - as opposed to the body derivative of velocity, vB_BI_dot)
        // That is, an accelerometer measures specific force - which is inertial acceleration minus the contribution from gravity and expressed in the body frame
    };

    struct Gyroscope : Sensor {
        std::optional<Eigen::Vector3d> prev_wB_BI_lag;
        AngularVelocityMeasurement measure(const dynamics::AngularVelocity& wB_BI, double dt);
    };

    struct PitotTube : Sensor {
        std::optional<double> prev_P0_lag;
        StagnationAirPressureMeasurement measure(const atmospheric::StagnationAirPressure& P0, double dt);
    };

    struct StaticPort : Sensor {
        std::optional<double> prev_P_lag;
        StaticAirPressureMeasurement measure(const atmospheric::StaticAirPressure& P, double dt);
    };

    struct TotalAirTemperatureProbe : Sensor {
        std::optional<double> prev_T0_lag;
        StagnationAirTemperatureMeasurement measure(const atmospheric::StagnationAirTemperature& T0, double dt);
    };

    struct GNSSReceiver : Sensor {
        std::optional<Eigen::Vector3d> prev_pI_BI_lag;
        std::optional<Eigen::Vector3d> prev_vB_BI_lag;
        PositionMeasurement measure(const dynamics::Position& pI_BI, double dt);
        TranslationalVelocityMeasurement measure(const dynamics::TranslationalVelocity& vB_BI, double dt);
    };

    struct Magnetometer : Sensor {
        std::optional<double> prev_heading_lag;
        HeadingMeasurement measure(const geography::Heading& heading, double dt);
    };

    struct SensorGroundTruth {
        const aerodynamics::AngleOfAttack alpha;
        const dynamics::TranslationalAcceleration fB;
        const dynamics::AngularVelocity wB_BI;
        const atmospheric::StagnationAirPressure P0;
        const atmospheric::StaticAirPressure P;
        const atmospheric::StagnationAirTemperature T0;
        const dynamics::Position pI_BI;
        const dynamics::TranslationalVelocity vB_BI;
        const geography::Heading heading;
    };

    struct SensorMeasurements {
        AngleOfAttackMeasurement alpha;
        TranslationalAccelerationMeasurement fB;
        AngularVelocityMeasurement wB_BI;
        StagnationAirPressureMeasurement P0;
        StaticAirPressureMeasurement P;
        StagnationAirTemperatureMeasurement T0;
        PositionMeasurement pI_BI_gnss;
        TranslationalVelocityMeasurement vB_BI_gnss;
        HeadingMeasurement heading_BE;
    };

    struct Sensors {
        AngleOfAttackVane aoa_vane;
        Accelerometer accelerometer; // IMU
        Gyroscope gyro; // IMU
        PitotTube pitot_tube;
        StaticPort static_port;
        TotalAirTemperatureProbe tat_probe;
        GNSSReceiver gnss;
        Magnetometer magnetometer;
    };

    struct SensorManager {
        Sensors sensors;
        std::optional<SensorMeasurements> hist;

        SensorMeasurements step(const SensorGroundTruth& sensor_gt, double dt);
    };

    SensorGroundTruth build_sensor_gt(
        const dynamics::RigidBodyState& Xt,
        const dynamics::RigidBodyState& XEt,
        const aerodynamics::AerodynamicState& aero_t,
        const atmospheric::StaticAtmosphericState& atm_t,
        const dynamics::Mass& mass,
        const dynamics::Wrench& WB_net
    );

}
