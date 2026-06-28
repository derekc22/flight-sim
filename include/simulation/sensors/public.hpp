#pragma once
#include <optional>
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/sensors/private.hpp"

namespace sensors {

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

    struct SensorProperties {
        Sensors sensors;
        std::optional<SensorMeasurements> hist;

        SensorMeasurements step(const SensorGroundTruth& sensor_gt);
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
