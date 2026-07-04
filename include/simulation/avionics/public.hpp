#pragma once
#include <Eigen/Dense>
#include <optional>
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/sensors/public.hpp"

namespace avionics {

    // computed
    struct StaticAirTemperatureMeasurement : atmospheric::StaticAirTemperature {};

    struct MachNumberMeasurement : atmospheric::MachNumber {};

    struct OrientationMeasurement : dynamics::OrientationQuaternion {};

    struct FreeStreamVelocityMeasurement : aerodynamics::FreeStreamVelocity {};

    struct PressureAltitudeMeasurement : geography::PressureAltitude {};

    struct VerticalSpeedMeasurement : dynamics::VerticalSpeed {};

    struct AirDensityMeasurement : atmospheric::AirDensity {};

    struct AirDataComputer {
        FreeStreamVelocityMeasurement compute(const MachNumberMeasurement& Mach, const StaticAirTemperatureMeasurement& T);
        PressureAltitudeMeasurement compute(const sensors::StaticAirPressureMeasurement& P);
        VerticalSpeedMeasurement compute(const sensors::StaticAirPressureMeasurement& P, const sensors::StaticAirPressureMeasurement& prev_P, const StaticAirTemperatureMeasurement& T, double dt);
        AirDensityMeasurement compute(const sensors::StaticAirPressureMeasurement& P, const StaticAirTemperatureMeasurement& T);
    };

    struct AttitudeHeadingReferenceSystem {
        OrientationMeasurement compute(const OrientationMeasurement& prev_qIB, const sensors::AngularVelocityMeasurement& wB_BI, const sensors::TranslationalAccelerationMeasurement& fB, double dt);
        double Kp;
        double Ki;
        double fB_tol_scale;

        Eigen::Vector3d integral = constants::Zero3;
        double fB_tol = fB_tol_scale * constants::g_earth;
    };

    struct InertialNavigationSystem {
        sensors::PositionMeasurement compute(const sensors::PositionMeasurement& prev_pI_BI, const sensors::TranslationalVelocityMeasurement& prev_vB_BI, const sensors::TranslationalAccelerationMeasurement& fB, const dynamics::Gravity& gB, const OrientationMeasurement& prev_qIB, double dt);
        sensors::TranslationalVelocityMeasurement compute(const sensors::TranslationalVelocityMeasurement& prev_vB_BI, const sensors::TranslationalAccelerationMeasurement& fB, const dynamics::Gravity& gB, const sensors::AngularVelocityMeasurement& wB_BI, double dt);
    };

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
            const AvionicsGroundTruth& avionics_gt,
            double dt
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
