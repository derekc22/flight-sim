#pragma once
#include <Eigen/Dense>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/geography/geography.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"

namespace sensors {

    struct PositionMeasurement : dynamics::Position {};

    struct OrientationMeasurement : dynamics::OrientationQuaternion {};

    struct LinearVelocityMeasurement : dynamics::LinearVelocity {};

    struct LinearAccelerationMeasurement : dynamics::LinearAcceleration {};

    struct AngularVelocityMeasurement : dynamics::AngularVelocity {};

    struct FreeStreamVelocityMeasurement : aerodynamics::FreeStreamVelocity {};

    struct AngleOfAttackMeasurement : aerodynamics::AngleOfAttack {};

    struct AltitudeMeasurement : geography::Altitude {};

    // struct StaticAirTemperatureMeasurement : atmospheric::StaticAirTemperature {};

    struct StagnationAirTemperatureMeasurement : atmospheric::StagnationAirTemperature {};

    struct AirDensityMeasurement : atmospheric::AirDensity {};

    struct StaticAirPressureMeasurement : atmospheric::StaticAirPressure {};

    struct StagnationAirPressureMeasurement : atmospheric::StagnationAirPressure {};

    struct VerticalSpeedMeasurement : dynamics::VerticalSpeed {};

    struct MagneticFieldVectorMeasurement : geography::MagneticFieldVector {};

    struct HeadingMeasurement : geography::Heading {};

    // Note: Due to modelling complexity, the internal dynamics of some sensors will remain unmodelled
    // Instead, these sensor will simply output a noised, delayed version of the corresponding ground-truth quantity

    struct Sensor {
        double mean = 0.0;
        double cov = 0.0;
        double tau = constants::eps;
    };

    struct AngleOfAttackVane : Sensor {
        AngleOfAttackMeasurement measure(aerodynamics::AngleOfAttack AoA);
    };

    // IMU
    struct Accelerometer : Sensor {
        LinearAccelerationMeasurement measure(dynamics::LinearAcceleration accel);
        // Note: accel = F_non-gravity/m - g, not pB_BI_ddot
        // That is, an accelerometer measures all forces/accelerations, excluding gravity 
    };

    // IMU
    struct Gyroscope : Sensor {
        AngularVelocityMeasurement measure(dynamics::AngularVelocity wB_BI);
    };

    struct PitotTube : Sensor {
        StagnationAirPressureMeasurement measure(atmospheric::StagnationAirPressure P0);
    };

    struct StaticPort : Sensor {
        StaticAirPressureMeasurement measure(atmospheric::StaticAirPressure P);
    };

    struct TotalAirTemperatureProbe : Sensor {
        StagnationAirTemperatureMeasurement measure(atmospheric::StagnationAirTemperature);
    };

    struct GNSSReceiver : Sensor {
        PositionMeasurement measure(dynamics::Position pI_BI);
        LinearVelocityMeasurement measure(dynamics::LinearVelocity pB_BI_dot);
    }

    struct Magnetometer : Sensor {
        MagneticFieldVectorMeasurement measure(geography::MagneticFieldVector magE);
    };

    

}
