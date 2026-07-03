#pragma once
#include <Eigen/Dense>
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/sensors/private.hpp"


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
        FreeStreamVelocityMeasurement compute(
            const MachNumberMeasurement& Mach, 
            const StaticAirTemperatureMeasurement& T
        );
        PressureAltitudeMeasurement compute(
            const sensors::StaticAirPressureMeasurement& P
        );
        VerticalSpeedMeasurement compute(
            const sensors::StaticAirPressureMeasurement& P, 
            const sensors::StaticAirPressureMeasurement& prev_P, 
            const StaticAirTemperatureMeasurement& T
        );
        AirDensityMeasurement compute(
            const sensors::StaticAirPressureMeasurement& P,     
            const StaticAirTemperatureMeasurement& T
        );
    };

    struct AttitudeHeadingReferenceSystem {
        OrientationMeasurement compute(
            const OrientationMeasurement& prev_qIB, 
            const sensors::AngularVelocityMeasurement& wB_BI,
            const sensors::TranslationalAccelerationMeasurement& fB
        );
        double Kp;
        double Ki;
        Eigen::Vector3d integral = constants::Zero3;
        double fB_tol = 0.2 * constants::g_earth;
    };

    struct InertialNavigationSystem {
        sensors::PositionMeasurement compute(
            const sensors::PositionMeasurement& prev_pI_BI, 
            const sensors::TranslationalVelocityMeasurement& prev_vB_BI, 
            const sensors::TranslationalAccelerationMeasurement& fB, 
            const dynamics::Gravity& gB, 
            const OrientationMeasurement& prev_qIB
        );
        sensors::TranslationalVelocityMeasurement compute(
            const sensors::TranslationalVelocityMeasurement& prev_vB_BI, 
            const sensors::TranslationalAccelerationMeasurement& fB,
            const dynamics::Gravity& gB, 
            const sensors::AngularVelocityMeasurement& wB_BI
        );
    };
}
