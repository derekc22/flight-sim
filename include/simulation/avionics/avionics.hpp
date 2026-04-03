#pragma once
#include "simulation/sensors/sensors.hpp"

namespace avionics {

    struct AirDataComputer {
        sensors::FreeStreamVelocityMeasurement calculate(sensors::StagnationAirPressureMeasurement P0_meas, sensors::StaticAirPressureMeasurement P_meas);
        sensors::AltitudeMeasurement calculate(sensors::StaticAirPressureMeasurement P_meas);
        sensors::VerticalSpeedMeasurement calculate(sensors::StaticAirPressureMeasurement P_meas);
        sensors::AirDensityMeasurement calculate(sensors::StaticAirPressureMeasurement P_meas, sensors::StagnationAirTemperatureMeasurement T0_meas);
    };

    struct AttitudeHeadingReferenceSystem {
        sensors::OrientationMeasurement calculate(sensors::LinearAccelerationMeasurement accel_meas, sensors::AngularVelocityMeasurement wB_BI_meas);
        sensors::HeadingMeasurement calculate(sensors::MagneticFieldVectorMeasurement mag_meas, sensors::OrientationMeasurement qIB_meas);
    };

    struct InertiallNavigationSystem {
        // outputs position, velocity

    }


}
