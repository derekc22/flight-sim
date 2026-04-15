#pragma once
#include <Eigen/Dense>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/geography/geography.hpp"
#include "simulation/avionics/sensors.hpp"


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
        LinearVelocityMeasurement vB_BI_ins;
        StaticAirTemperatureMeasurement T;
        MachNumberMeasurement Mach;
        OrientationMeasurement qIB;
        FreeStreamVelocityMeasurement Vinf;
        AltitudeMeasurement alt_BE;
        VerticalSpeedMeasurement alt_BE_dot;
        AirDensityMeasurement rho;
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
}
