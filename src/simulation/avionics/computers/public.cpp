#include <cmath>
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/avionics/computers/public.hpp"
#include "simulation/util/public.hpp"

namespace avionics {

    FreeStreamVelocityMeasurement AirDataComputer::compute(const MachNumberMeasurement& Mach, const StaticAirTemperatureMeasurement& T) {
        double Vinf = Mach.data * util::sqrt(constants::gamma_air * constants::R_air * T.data);
        return { Vinf };
    }

    PressureAltitudeMeasurement AirDataComputer::compute(const StaticAirPressureMeasurement& P) {
        double pressure_alt = (constants::T_SL / std::abs(constants::lapse_rate)) * (1.0 - std::pow(P.data / constants::P_SL, (constants::R_air * std::abs(constants::lapse_rate)) / constants::g_earth));
        return { pressure_alt };
    }

    VerticalSpeedMeasurement AirDataComputer::compute(const StaticAirPressureMeasurement& P, const StaticAirPressureMeasurement& prev_P, const StaticAirTemperatureMeasurement& T) {
        double P_dot_meas = (P.data - prev_P.data) / constants::dt;
        double alt_BE_dot = -(constants::R_air * T.data / (constants::g_earth * P.data)) * P_dot_meas;
        return { alt_BE_dot };
    }

    AirDensityMeasurement AirDataComputer::compute(const StaticAirPressureMeasurement& P, const StaticAirTemperatureMeasurement& T) {
        double rho = P.data / (constants::R_air * T.data);
        return { rho };
    }

    OrientationMeasurement AttitudeHeadingReferenceSystem::compute(const OrientationMeasurement& prev_qIB, const AngularVelocityMeasurement& wB_BI) {
        return { dynamics::quat_kin(prev_qIB, wB_BI) };
    }

    PositionMeasurement InertialNavigationSystem::compute(
        const PositionMeasurement& prev_pI_BI, 
        const TranslationalVelocityMeasurement& prev_vB_BI, 
        const TranslationalAccelerationMeasurement& fB, 
        const dynamics::Gravity& gB, 
        const OrientationMeasurement& prev_qIB
    ) {
        dynamics::TranslationalVelocity pI_BI_dot{ prev_qIB.data.conjugate() * prev_vB_BI.data };
        dynamics::TranslationalAcceleration aI_BI{ prev_qIB.data.conjugate() * (fB.data + gB.data) };
        return { dynamics::trans_kin(prev_pI_BI, pI_BI_dot, aI_BI).data };
    }

    TranslationalVelocityMeasurement InertialNavigationSystem::compute(
        const TranslationalVelocityMeasurement& prev_vB_BI, 
        const TranslationalAccelerationMeasurement& fB, 
        const dynamics::Gravity& gB, 
        const AngularVelocityMeasurement& wB_BI
    ) {
        dynamics::TranslationalAcceleration vB_BI_dot{ fB.data + gB.data - wB_BI.data.cross(prev_vB_BI.data) };
        return { dynamics::trans_kin_vel(prev_vB_BI, vB_BI_dot).data };
    }

}
