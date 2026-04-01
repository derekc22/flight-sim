#pragma once
#include <Eigen/Dense>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/geography/geography.hpp"

namespace sensors {

    struct PositionMeasurement : dynamics::Position {};

    struct LinearVelocityMeasurement : dynamics::LinearVelocity {};

    struct LinearAccelerationMeasurement : dynamics::LinearAcceleration {};

    struct AngularVelocityMeasurement : dynamics::AngularVelocity {};

    struct FreeStreamVelocityMeasurement : aerodynamics::FreeStreamVelocity {};

    struct AngleOfAttackMeasurement : aerodynamics::AngleOfAttack {};

    struct AltitudeMeasurement : geography::Altitude {};

    struct TemperatureMeasurement : atmospheric::Temperature {};

    struct DensityMeasurement : atmospheric::Density {};

    struct StagnationPressureMeasurement : atmospheric::StagnationPressure {};

    struct StaticPressureMeasurement : atmospheric::StaticPressure {};

    struct VerticalSpeedMeasurement : dynamics::VerticalSpeed {};

    // Under-modelled sensors:
    // Due to modelling complexity, the internal dynamics of some sensors will remain unmodelled
    // Instead, these sensor will simply output a noised version of the corresponding ground-truth state

    struct Sensor {
        double mean = 0.0;
        double cov = 0.0;
    };

    struct AngleOfAttackVane : Sensor {
        AngleOfAttackMeasurement measure(aerodynamics::AngleOfAttack AoA);
    };

    struct Accelerometer : Sensor {
        LinearAccelerationMeasurement measure(dynamics::LinearAcceleration pB_BI_ddot);
    };

    struct Gyroscope : Sensor {
        AngularVelocityMeasurement measure(dynamics::AngularVelocity wB_BI);
    };

    struct PitotTube : Sensor {
        StagnationPressureMeasurement measure(atmospheric::StagnationPressure P_total);
    };

    struct StaticPort : Sensor {
        StaticPressureMeasurement measure(atmospheric::StaticPressure P_static);
    };

    struct TotalAirTemperatureProbe : Sensor {
        TemperatureMeasurement measure(...);
    };

    struct GNSSPositionReceiver : Sensor {
        PositionMeasurement measure(dynamics::Position pI_BI);
    };

    struct GNSSVelocityReceiver : Sensor {
        LinearVelocityMeasurement measure(dynamics::LinearVelocity pB_BI_dot);
    };


    // Computers
    struct AirDataComputer {
        FreeStreamVelocityMeasurement measure(...);
        AltitudeMeasurement measure(...);
        VerticalSpeedMeasurement measure(...);
        DensityMeasurement measure();
    };




}