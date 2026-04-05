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
        dynamics::LinearAcceleration accel;
        dynamics::AngularVelocity wB_BI;
        atmospheric::StagnationAirPressure P0;
        atmospheric::StaticAirPressure P;
        atmospheric::StagnationAirTemperature T0;
        dynamics::Position pI_BI;
        dynamics::LinearVelocity pB_BI_dot;

        atmospheric::StaticAirTemperature T;
        atmospheric::MachNumber Mach;
        geography::Heading heading;
        dynamics::OrientationQuaternion qIB;
        aerodynamics::FreeStreamVelocity Vinf;
        geography::Altitude alt_BE;
        dynamics::VerticalSpeed alt_dot;
        atmospheric::AirDensity rho;
    };

    struct MeasurementCache {
        AngleOfAttackMeasurement curr_alpha_meas;
        LinearAccelerationMeasurement curr_accel_meas;
        AngularVelocityMeasurement curr_wB_BI_meas;
        StagnationAirPressureMeasurement curr_P0_meas;
        StaticAirPressureMeasurement curr_P_meas;
        StagnationAirTemperatureMeasurement curr_T0_meas;
        PositionMeasurement curr_pI_BI_gnss_meas;
        LinearVelocityMeasurement curr_pB_BI_dot_gnss_meas;
        HeadingMeasurement curr_heading_BE_meas;

        PositionMeasurement curr_pI_BI_ins_meas;
        LinearVelocityMeasurement curr_pB_BI_dot_ins_meas;
        StaticAirTemperatureMeasurement curr_T_meas;
        MachNumberMeasurement curr_Mach_meas;
        OrientationMeasurement curr_qIB_meas;
        FreeStreamVelocityMeasurement curr_Vinf_meas;
        AltitudeMeasurement curr_alt_BE_meas;
        VerticalSpeedMeasurement curr_alt_BE_dot_meas;
        AirDensityMeasurement curr_rho_meas;

        void update_from_sensor_cache(const MeasurementCache& sensor_cache);
        void update_from_computer_cache(const MeasurementCache& computer_cache);
    };

    struct MeasurementHistory {
        std::optional<AngleOfAttackMeasurement> prev_alpha_meas;
        std::optional<LinearAccelerationMeasurement> prev_accel_meas;
        std::optional<AngularVelocityMeasurement> prev_wB_BI_meas;
        std::optional<StagnationAirPressureMeasurement> prev_P0_meas;
        std::optional<StaticAirPressureMeasurement> prev_P_meas;
        std::optional<StagnationAirTemperatureMeasurement> prev_T0_meas;
        std::optional<PositionMeasurement> prev_pI_BI_gnss_meas;
        std::optional<LinearVelocityMeasurement> prev_pB_BI_dot_gnss_meas;
        std::optional<HeadingMeasurement> prev_heading_BE_meas;

        std::optional<PositionMeasurement> prev_pI_BI_ins_meas;
        std::optional<LinearVelocityMeasurement> prev_pB_BI_dot_ins_meas;
        std::optional<StaticAirTemperatureMeasurement> prev_T_meas;
        std::optional<MachNumberMeasurement> prev_Mach_meas;
        std::optional<OrientationMeasurement> prev_qIB_meas;
        std::optional<FreeStreamVelocityMeasurement> prev_Vinf_meas;
        std::optional<AltitudeMeasurement> prev_alt_BE_meas;
        std::optional<VerticalSpeedMeasurement> prev_alt_dot_meas;
        std::optional<AirDensityMeasurement> prev_rho_meas;
    };




    // Note: Due to modelling complexity, the internal dynamics of some sensors will remain unmodelled
    // Instead, these sensor will simply output a noised, delayed version of the corresponding ground-truth quantity

    struct Sensor {
        double mean;
        double stddev;
        // double bias;
        std::mt19937 gen{std::random_device{}()};
        std::normal_distribution<double> dist;

        double tau;
        double alpha;

        Sensor(double m, double s) : mean(m), stddev(s), dist(m, s) {}

        double _step(double meas, double prev_meas);
        Eigen::Vector3d _step(const Eigen::Vector3d& meas, const Eigen::Vector3d& prev_meas);
        Eigen::Quaterniond _step(const Eigen::Quaterniond& meas, const Eigen::Quaterniond& prev_meas);
    };

    struct AngleOfAttackVane : Sensor {
        AngleOfAttackMeasurement _measure(const aerodynamics::AngleOfAttack& alpha, const aerodynamics::AngleOfAttack& prev_alpha);
    };

    struct Accelerometer : Sensor {
        LinearAccelerationMeasurement _measure(const dynamics::LinearAcceleration& accel, const dynamics::LinearAcceleration& prev_accel);
        // Note: accel = F_net/m - g, not pB_BI_ddot
        // That is, an accelerometer measures all accelerations excluding gravity 
    };

    struct Gyroscope : Sensor {
        AngularVelocityMeasurement _measure(const dynamics::AngularVelocity& wB_BI, const dynamics::AngularVelocity& prev_wB_BI);
    };

    struct PitotTube : Sensor {
        StagnationAirPressureMeasurement _measure(const atmospheric::StagnationAirPressure& P0, const atmospheric::StagnationAirPressure& prev_P0);
    };

    struct StaticPort : Sensor {
        StaticAirPressureMeasurement _measure(const atmospheric::StaticAirPressure& P, const atmospheric::StaticAirPressure& prev_P);
    };

    struct TotalAirTemperatureProbe : Sensor {
        StagnationAirTemperatureMeasurement _measure(const atmospheric::StagnationAirTemperature& T0, const atmospheric::StagnationAirTemperature& prev_T0);
    };

    struct GNSSReceiver : Sensor {
        PositionMeasurement _measure(const dynamics::Position& pI_BI, const dynamics::Position& prev_pI_BI);
        LinearVelocityMeasurement _measure(const dynamics::LinearVelocity& pB_BI_dot, const dynamics::LinearVelocity& prev_pB_BI_dot);
    };

    struct Magnetometer : Sensor {
        HeadingMeasurement _measure(const geography::Heading& heading, const geography::Heading& prev_heading);
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
        PositionMeasurement _calculate(const PositionMeasurement& prev_pI_BI, const LinearVelocityMeasurement& prev_pB_BI_dot, const LinearAccelerationMeasurement& accel, const OrientationMeasurement& prev_qIB);
        LinearVelocityMeasurement _calculate(const LinearVelocityMeasurement& prev_pB_BI_dot, const LinearAccelerationMeasurement& accel, const PositionMeasurement& prev_pI_BI, const OrientationMeasurement& prev_qIB, const AngularVelocityMeasurement& wB_BI);
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
        MeasurementHistory hist_from_cache(const MeasurementCache& cache);

    };


    std::pair<dynamics::RigidBodyState, aerodynamics::AerodynamicState> update_state_from_avionics(
        const dynamics::RigidBodyState& xN_t, 
        const aerodynamics::AerodynamicState& ads_t, 
        const atmospheric::StaticAtmosphericState& static_atmo_t, 
        const geography::GeographicState& geo_t,
        const dynamics::Mass& mass,
        const atmospheric::Wind & wind,
        const dynamics::Wrench WB_net,
        AvionicsProperties& avionics_properties
    );




}
