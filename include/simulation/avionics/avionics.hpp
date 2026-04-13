#pragma once
#include <Eigen/Dense>
#include <optional>
#include <random>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/geography/geography.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"

namespace avionics {

    // directly measured
    struct AngleOfAttackMeasurement : aerodynamics::AngleOfAttack {};

    struct LinearAccelerationMeasurement : dynamics::LinearAcceleration {};

    struct AngularVelocityMeasurement : dynamics::AngularVelocity {};

    struct StagnationAirPressureMeasurement : atmospheric::StagnationAirPressure {};

    struct StaticAirPressureMeasurement : atmospheric::StaticAirPressure {};

    struct StagnationAirTemperatureMeasurement : atmospheric::StagnationAirTemperature {};

    struct PositionMeasurement : dynamics::Position {};

    struct LinearVelocityMeasurement : dynamics::LinearVelocity {};

    struct HeadingMeasurement : geography::Heading {};


    // computed
    struct StaticAirTemperatureMeasurement : atmospheric::StaticAirTemperature {};

    struct MachNumberMeasurement : atmospheric::MachNumber {};

    struct OrientationMeasurement : dynamics::OrientationQuaternion {};

    struct FreeStreamVelocityMeasurement : aerodynamics::FreeStreamVelocity {};

    struct AltitudeMeasurement : geography::Altitude {};

    struct VerticalSpeedMeasurement : dynamics::VerticalSpeed {};

    struct AirDensityMeasurement : atmospheric::AirDensity {};


    struct MeasurementGroundTruth {
        aerodynamics::AngleOfAttack alpha;
        dynamics::LinearAcceleration accelB;
        dynamics::AngularVelocity wB_BI;
        atmospheric::StagnationAirPressure P0;
        atmospheric::StaticAirPressure P;
        atmospheric::StagnationAirTemperature T0;
        dynamics::Position pI_BI;
        dynamics::LinearVelocity vB_BI;

        atmospheric::StaticAirTemperature T;
        atmospheric::MachNumber Mach;
        geography::Heading heading;
        dynamics::OrientationQuaternion qIB;
        aerodynamics::FreeStreamVelocity Vinf;
        geography::Altitude alt_BE;
        dynamics::VerticalSpeed alt_dot;
        atmospheric::AirDensity rho;
    };

    struct SensorMeasurements {
        AngleOfAttackMeasurement alpha;
        LinearAccelerationMeasurement accel;
        AngularVelocityMeasurement wB_BI;
        StagnationAirPressureMeasurement P0;
        StaticAirPressureMeasurement P;
        StagnationAirTemperatureMeasurement T0;
        PositionMeasurement pI_BI_gnss;
        LinearVelocityMeasurement vB_BI_gnss;
        HeadingMeasurement heading_BE;
    };

    struct ComputerMeasurements {
        PositionMeasurement pI_BI_ins;
        LinearVelocityMeasurement vB_BI_ins;
        StaticAirTemperatureMeasurement T;
        MachNumberMeasurement Mach;
        OrientationMeasurement qIB;
        FreeStreamVelocityMeasurement Vinf;
        AltitudeMeasurement alt_BE;
        VerticalSpeedMeasurement alt_BE_dot;
        AirDensityMeasurement rho;
    };

    struct MeasurementCache {
        SensorMeasurements sensors;
        ComputerMeasurements computers;
    };

    struct MeasurementHistory {
        std::optional<SensorMeasurements> sensors;
        std::optional<ComputerMeasurements> computers;
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

        Sensor(double mean, double stddev, double bias, const Eigen::Vector3d& bias3, double tau);

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
        LinearAccelerationMeasurement _measure(const dynamics::LinearAcceleration& accelB);
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
        LinearVelocityMeasurement _measure(const dynamics::LinearVelocity& vB_BI);
    };

    struct Magnetometer : Sensor {
        std::optional<double> prev_heading_lag;
        HeadingMeasurement _measure(const geography::Heading& heading);
    };




    struct AirDataComputer {
        FreeStreamVelocityMeasurement _calculate(const MachNumberMeasurement& Mach, const StaticAirTemperatureMeasurement& T);
        AltitudeMeasurement _calculate(const StaticAirPressureMeasurement& P);
        VerticalSpeedMeasurement _calculate(const StaticAirPressureMeasurement& P, const StaticAirPressureMeasurement& prev_P, const StaticAirTemperatureMeasurement& T);
        AirDensityMeasurement _calculate(const StaticAirPressureMeasurement& P, const StaticAirTemperatureMeasurement& T);
    };

    struct AttitudeHeadingReferenceSystem {
        OrientationMeasurement _calculate(const OrientationMeasurement& prev_qIB, const AngularVelocityMeasurement& wB_BI);
    };

    struct InertialNavigationSystem {
        PositionMeasurement _calculate(const PositionMeasurement& prev_pI_BI, const LinearVelocityMeasurement& prev_vB_BI, const LinearAccelerationMeasurement& accelB, const OrientationMeasurement& prev_qIB);
        LinearVelocityMeasurement _calculate(const LinearVelocityMeasurement& prev_vB_BI, const LinearAccelerationMeasurement& accelB, const PositionMeasurement& prev_pI_BI, const OrientationMeasurement& prev_qIB, const AngularVelocityMeasurement& wB_BI);
    };


    struct AvionicsSensors {
        AngleOfAttackVane aoa_vane;
        Accelerometer accelerometer; // IMU
        Gyroscope gyro; // IMU
        PitotTube pitot_tube;
        StaticPort static_port;
        TotalAirTemperatureProbe tat_probe;
        GNSSReceiver gnss;
        Magnetometer magnetometer;
    };

    struct AvionicsComputers {
        AirDataComputer ADC;
        AttitudeHeadingReferenceSystem AHRS;
        InertialNavigationSystem INS;
    };

    struct AvionicsProperties {
        AvionicsSensors sensors;
        AvionicsComputers computers;
        MeasurementHistory hist;
        MeasurementCache cache;

        MeasurementCache step(const MeasurementGroundTruth& meas_gt);

    };


    // std::pair<dynamics::RigidBodyState, aerodynamics::AerodynamicState> get_state_from_avionics(
    dynamics::RigidBodyState get_state_from_avionics(
        const dynamics::RigidBodyState& xN_t, 
        const aerodynamics::AerodynamicState& ads_t, 
        const atmospheric::StaticAtmosphericState& static_atmo_t, 
        const geography::GeographicState& geo_t,
        const dynamics::Mass& mass,
        const atmospheric::Wind & wind,
        const dynamics::Wrench& WB_net,
        AvionicsProperties& avionics_properties
    );




}
