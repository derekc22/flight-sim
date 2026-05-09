#pragma once
#include <Eigen/Dense>
#include <optional>
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/geography/geography.hpp"
#include "simulation/avionics/sensors.hpp"
#include "simulation/avionics/computers.hpp"

namespace avionics {

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

    struct MeasurementCache {
        SensorMeasurements sensors;
        ComputerMeasurements computers;
    };

    struct MeasurementHistory {
        std::optional<SensorMeasurements> sensors;
        std::optional<ComputerMeasurements> computers;
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
        const atmospheric::StaticAtmosphericState& static_atm_t, 
        const geography::GeographicState& gps_t,
        const dynamics::Mass& mass,
        const atmospheric::Wind & wind,
        const dynamics::Wrench& WB_net,
        AvionicsProperties& avionics_properties
    );

}
