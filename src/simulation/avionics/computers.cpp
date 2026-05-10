#include <cmath>
#include "simulation/constants/constants.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/geography/shared.hpp"
#include "simulation/avionics/computers.hpp"

namespace avionics {

    FreeStreamVelocityMeasurement AirDataComputer::_calculate(const MachNumberMeasurement& Mach, const StaticAirTemperatureMeasurement& T) {
        double Vinf = Mach.data * std::sqrt(constants::gamma_air * constants::R_air * T.data);
        return { Vinf };
    }

    AltitudeMeasurement AirDataComputer::_calculate(const StaticAirPressureMeasurement& P) {
        double alt = (constants::T_SL / std::abs(constants::lapse_rate)) * (1.0 - std::pow(P.data / constants::P_SL, (constants::R_air * std::abs(constants::lapse_rate)) / constants::g_earth));
        return { alt };
    }

    VerticalSpeedMeasurement AirDataComputer::_calculate(const StaticAirPressureMeasurement& P, const StaticAirPressureMeasurement& prev_P, const StaticAirTemperatureMeasurement& T) {
        double P_dot_meas = (P.data - prev_P.data) / constants::dt;
        double alt_dot = -(constants::R_air * T.data / (constants::g_earth * P.data)) * P_dot_meas;
        return { alt_dot };
    }

    AirDensityMeasurement AirDataComputer::_calculate(const StaticAirPressureMeasurement& P, const StaticAirTemperatureMeasurement& T) {
        double rho = P.data / (constants::R_air * T.data);
        return { rho };
    }

    OrientationMeasurement AttitudeHeadingReferenceSystem::_calculate(const OrientationMeasurement& prev_qIB, const AngularVelocityMeasurement& wB_BI) {
        return { dynamics::_quat_kin(prev_qIB, wB_BI) };
    }

    PositionMeasurement InertialNavigationSystem::_calculate(const PositionMeasurement& prev_pI_BI, const TranslationalVelocityMeasurement& prev_vB_BI, const TranslationalAccelerationMeasurement& accelB, const OrientationMeasurement& prev_qIB) {
        dynamics::Gravity prev_gB = geography::gB(prev_pI_BI, prev_qIB);
        dynamics::TranslationalVelocity pI_BI_dot{ prev_qIB.data.conjugate() * prev_vB_BI.data };
        dynamics::TranslationalAcceleration vI_BI_dot{ prev_qIB.data.conjugate() * (accelB.data + prev_gB.data) };
        return { dynamics::_trans_kin(prev_pI_BI, pI_BI_dot, vI_BI_dot).data };
    }

    TranslationalVelocityMeasurement InertialNavigationSystem::_calculate(const TranslationalVelocityMeasurement& prev_vB_BI, const TranslationalAccelerationMeasurement& accelB, const PositionMeasurement& prev_pI_BI, const OrientationMeasurement& prev_qIB, const AngularVelocityMeasurement& wB_BI) {
        dynamics::Gravity prev_gB = geography::gB(prev_pI_BI, prev_qIB);
        dynamics::TranslationalAcceleration vB_BI_dot{ accelB.data + prev_gB.data - wB_BI.data.cross(prev_vB_BI.data) };
        return { dynamics::_trans_kin_vel(prev_vB_BI, vB_BI_dot).data };
    }

}
