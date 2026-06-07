#pragma once
#include <Eigen/Dense>
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/avionics/sensors/public.hpp"


namespace avionics {

    // computed
    struct StaticAirTemperatureMeasurement : atmospheric::StaticAirTemperature {};

    struct MachNumberMeasurement : atmospheric::MachNumber {};

    struct OrientationMeasurement : dynamics::OrientationQuaternion {};

    struct FreeStreamVelocityMeasurement : aerodynamics::FreeStreamVelocity {};

    struct AltitudeMeasurement : geography::Altitude {};

    struct VerticalSpeedMeasurement : dynamics::VerticalSpeed {};

    struct AirDensityMeasurement : atmospheric::AirDensity {};


    struct ComputerMeasurements {
        PositionMeasurement pI_BI_ins;
        TranslationalVelocityMeasurement vB_BI_ins;
        StaticAirTemperatureMeasurement T;
        MachNumberMeasurement Mach;
        OrientationMeasurement qIB;
        FreeStreamVelocityMeasurement Vinf;
        AltitudeMeasurement alt_BE;
        VerticalSpeedMeasurement alt_BE_dot;
        AirDensityMeasurement rho;
    };

    struct AirDataComputer {
        FreeStreamVelocityMeasurement compute(const MachNumberMeasurement& Mach, const StaticAirTemperatureMeasurement& T);
        AltitudeMeasurement compute(const StaticAirPressureMeasurement& P);
        VerticalSpeedMeasurement compute(const StaticAirPressureMeasurement& P, const StaticAirPressureMeasurement& prev_P, const StaticAirTemperatureMeasurement& T);
        AirDensityMeasurement compute(const StaticAirPressureMeasurement& P, const StaticAirTemperatureMeasurement& T);
    };

    struct AttitudeHeadingReferenceSystem {
        OrientationMeasurement compute(const OrientationMeasurement& prev_qIB, const AngularVelocityMeasurement& wB_BI);
    };

    struct InertialNavigationSystem {
        PositionMeasurement compute(const PositionMeasurement& prev_pI_BI, const TranslationalVelocityMeasurement& prev_vB_BI, const TranslationalAccelerationMeasurement& fB, const dynamics::Gravity& gB, const OrientationMeasurement& prev_qIB);
        TranslationalVelocityMeasurement compute(const TranslationalVelocityMeasurement& prev_vB_BI, const TranslationalAccelerationMeasurement& fB, const dynamics::Gravity& gB, const AngularVelocityMeasurement& wB_BI);
    };
}
