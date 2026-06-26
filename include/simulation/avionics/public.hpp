#pragma once
#include <Eigen/Dense>
#include <optional>
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/avionics/sensors/public.hpp"
#include "simulation/avionics/computers/public.hpp"

namespace avionics {

    struct MeasurementGroundTruth {
        const aerodynamics::AngleOfAttack alpha;
        const dynamics::TranslationalAcceleration fB;
        const dynamics::Gravity gB;
        const dynamics::AngularVelocity wB_BI;
        const atmospheric::StagnationAirPressure P0;
        const atmospheric::StaticAirPressure P;
        const atmospheric::StagnationAirTemperature T0;
        const dynamics::Position pI_BI;
        const dynamics::TranslationalVelocity vB_BI;

        const atmospheric::StaticAirTemperature T;
        const atmospheric::MachNumber Mach;
        const geography::Heading heading;
        const dynamics::OrientationQuaternion qIB;
        const aerodynamics::FreeStreamVelocity Vinf;
        const geography::GeometricAltitude alt_BE;
        const dynamics::VerticalSpeed alt_BE_dot;
        const atmospheric::AirDensity rho;
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

	struct Settings {
        bool use_gnss = false;
	};

    struct AvionicsProperties {
        AvionicsSensors sensors;
        AvionicsComputers computers;
        MeasurementHistory hist;
        MeasurementCache cache;

        MeasurementCache step(const MeasurementGroundTruth& meas_gt);

        Settings settings;
    };

    MeasurementGroundTruth build_measurement_gt(
        const dynamics::RigidBodyState& Xt,
        const dynamics::RigidBodyState& XEt,
        const aerodynamics::AerodynamicState& aero_t,
        const atmospheric::StaticAtmosphericState& atm_t,
        const geography::GeographicState& geo_t,
        const dynamics::Mass& mass,
        const atmospheric::Wind & wind,
        const dynamics::Wrench& WB_net
    );

    dynamics::RigidBodyState get_state_from_avionics(const MeasurementCache& cache, const Settings& avionics_settings);
}
