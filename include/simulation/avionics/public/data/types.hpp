#pragma once
#include <optional>
#include "simulation/aerodynamics/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/geography/public/data/types.hpp"
#include "simulation/sensors/public/data/types.hpp"

namespace avionics {

    // computed
    struct StaticAirTemperatureMeasurement : atmospheric::StaticAirTemperature {};

    struct MachNumberMeasurement : atmospheric::MachNumber {};

    struct OrientationMeasurement : dynamics::OrientationQuaternion {};

    struct FreeStreamVelocityMeasurement : aerodynamics::FreeStreamVelocity {};

    struct PressureAltitudeMeasurement : geography::PressureAltitude {};

    struct VerticalSpeedMeasurement : dynamics::VerticalSpeed {};

    struct AirDensityMeasurement : atmospheric::AirDensity {};

    struct AvionicsGroundTruth {
        const dynamics::Gravity gB;
        const atmospheric::StaticAirTemperature T;
        const atmospheric::MachNumber Mach;
        const dynamics::OrientationQuaternion qIB;
        const aerodynamics::FreeStreamVelocity Vinf;
        const geography::GeometricAltitude alt_BE;
        const dynamics::VerticalSpeed alt_BE_dot;
        const atmospheric::AirDensity rho;
    };

    struct AvionicsMeasurements {
        sensors::PositionMeasurement pI_BI_ins;
        sensors::TranslationalVelocityMeasurement vB_BI_ins;
        StaticAirTemperatureMeasurement T;
        MachNumberMeasurement Mach;
        OrientationMeasurement qIB;
        FreeStreamVelocityMeasurement Vinf;
        PressureAltitudeMeasurement pressure_alt_BE;
        VerticalSpeedMeasurement alt_BE_dot;
        AirDensityMeasurement rho;
    };

	struct Settings {
        bool use_ins = false;
	};

    struct AvionicsManagerInput {
        const sensors::SensorMeasurements& sensor_measurements;
        const std::optional<sensors::SensorMeasurements>& previous_sensor_measurements;
        const sensors::SensorGroundTruth& sensor_ground_truth;
        const AvionicsGroundTruth& ground_truth;
        double dt;
    };

    struct AvionicsManagerOutput {
        AvionicsMeasurements measurements;
    };

}
