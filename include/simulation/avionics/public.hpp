#pragma once
#include <optional>
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/avionics/private.hpp"
#include "simulation/sensors/public.hpp"

namespace avionics {

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

    struct Avionics {
        AirDataComputer ADC;
        AttitudeHeadingReferenceSystem AHRS;
        InertialNavigationSystem INS;
    };

	struct Settings {
        bool use_ins = false;
	};

    struct AvionicsProperties {
        Avionics avionics;
        std::optional<AvionicsMeasurements> hist;

        AvionicsMeasurements step(
            const sensors::SensorMeasurements& sensor_meas,
            const std::optional<sensors::SensorMeasurements> sensor_hist,
            const sensors::SensorGroundTruth& sensor_gt,
            const AvionicsGroundTruth& avionics_gt
        );

        Settings settings;
    };

    AvionicsGroundTruth build_avionics_gt(
        const dynamics::RigidBodyState& Xt,
        const dynamics::RigidBodyState& XEt,
        const aerodynamics::AerodynamicState& aero_t,
        const atmospheric::StaticAtmosphericState& atm_t,
        const geography::GeographicState& geo_t
    );

    dynamics::RigidBodyState get_state_from_avionics(
        const sensors::SensorMeasurements& sensor_meas, 
        const AvionicsMeasurements& avionics_meas, 
        const Settings& avionics_settings
    );

}
