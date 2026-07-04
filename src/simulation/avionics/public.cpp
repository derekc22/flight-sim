#include <cmath>
#include <optional>
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/avionics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/sensors/public.hpp"
#include "simulation/util/public.hpp"

namespace avionics {

    FreeStreamVelocityMeasurement AirDataComputer::compute(const MachNumberMeasurement& Mach, const StaticAirTemperatureMeasurement& T) {
        double Vinf = Mach.data * util::sqrt(constants::gamma_air * constants::R_air * T.data);
        return { Vinf };
    }

    PressureAltitudeMeasurement AirDataComputer::compute(const sensors::StaticAirPressureMeasurement& P) {
        double pressure_alt = (constants::T_SL / std::abs(constants::lapse_rate)) * (1.0 - std::pow(P.data / constants::P_SL, (constants::R_air * std::abs(constants::lapse_rate)) / constants::g_earth));
        return { pressure_alt };
    }

    VerticalSpeedMeasurement AirDataComputer::compute(const sensors::StaticAirPressureMeasurement& P, const sensors::StaticAirPressureMeasurement& prev_P, const StaticAirTemperatureMeasurement& T, double dt) {
        double P_dot_meas = (P.data - prev_P.data) / dt;
        double alt_BE_dot = -(constants::R_air * T.data / (constants::g_earth * P.data)) * P_dot_meas;
        return { alt_BE_dot };
    }

    AirDensityMeasurement AirDataComputer::compute(const sensors::StaticAirPressureMeasurement& P, const StaticAirTemperatureMeasurement& T) {
        double rho = P.data / (constants::R_air * T.data);
        return { rho };
    }

    OrientationMeasurement AttitudeHeadingReferenceSystem::compute(const OrientationMeasurement& prev_qIB, const sensors::AngularVelocityMeasurement& wB_BI, const sensors::TranslationalAccelerationMeasurement& fB, double dt) {
        // Mahony filter
        dynamics::OrientationQuaternion qIB_pred = dynamics::quat_kin(prev_qIB, wB_BI, dt);
        if (util::abs(fB.data.norm() - constants::g_earth) > fB_tol) {
            return { qIB_pred };
        }

        Eigen::Vector3d gB_pred = qIB_pred.data * geography::gN().data;
        Eigen::Vector3d gB_meas = -fB.data;
        Eigen::Vector3d err = -gB_pred.normalized().cross(gB_meas.normalized());

        integral += err * dt;
        dynamics::AngularVelocity wB_BI_corrected{ wB_BI.data + Kp * err + Ki * integral };

        return { dynamics::quat_kin(prev_qIB, wB_BI_corrected, dt) };

    }

    sensors::PositionMeasurement InertialNavigationSystem::compute(const sensors::PositionMeasurement& prev_pI_BI, const sensors::TranslationalVelocityMeasurement& prev_vB_BI, const sensors::TranslationalAccelerationMeasurement& fB, const dynamics::Gravity& gB, const OrientationMeasurement& prev_qIB, double dt) {
        dynamics::TranslationalVelocity pI_BI_dot{ prev_qIB.data.conjugate() * prev_vB_BI.data };
        dynamics::TranslationalAcceleration aI_BI{ prev_qIB.data.conjugate() * (fB.data + gB.data) };
        return { dynamics::trans_kin(prev_pI_BI, pI_BI_dot, aI_BI, dt).data };
    }

    sensors::TranslationalVelocityMeasurement InertialNavigationSystem::compute(const sensors::TranslationalVelocityMeasurement& prev_vB_BI, const sensors::TranslationalAccelerationMeasurement& fB, const dynamics::Gravity& gB, const sensors::AngularVelocityMeasurement& wB_BI, double dt) {
        dynamics::TranslationalAcceleration vB_BI_dot{ fB.data + gB.data - wB_BI.data.cross(prev_vB_BI.data) };
        return { dynamics::trans_kin_vel(prev_vB_BI, vB_BI_dot, dt).data };
    }

    AvionicsMeasurements AvionicsProperties::step(
        const sensors::SensorMeasurements& sensor_meas,
        // sensor_hist will always be populated by the time AvionicsProperties::step is called, but std::optional is kept here for consistency
        const std::optional<sensors::SensorMeasurements> sensor_hist,
        const sensors::SensorGroundTruth& sensor_gt,
        const AvionicsGroundTruth& avionics_gt,
        double dt
    ) {
        MachNumberMeasurement mach_meas{ atmospheric::compute_mach(sensor_meas.P0, sensor_meas.P) };
        StaticAirTemperatureMeasurement T_meas{ atmospheric::T_from_T0(sensor_meas.T0, mach_meas) };

        AvionicsMeasurements avionics_meas {
            .pI_BI_ins = hist ?
                         avionics.INS.compute(
                            hist->pI_BI_ins,
                            hist->vB_BI_ins,
                            sensor_meas.fB,
                            avionics_gt.gB,
                            hist->qIB,
                            dt
                        ) : sensors::PositionMeasurement{ sensor_gt.pI_BI },
            .vB_BI_ins = hist ?
                         avionics.INS.compute(
                            hist->vB_BI_ins,
                            sensor_meas.fB,
                            avionics_gt.gB,
                            sensor_meas.wB_BI,
                            dt
                        ) : sensors::TranslationalVelocityMeasurement{ sensor_gt.vB_BI },
            .T = T_meas,
            .Mach = mach_meas,
            .qIB = hist ?
                   avionics.AHRS.compute(
                        hist->qIB,
                        sensor_meas.wB_BI,
                        sensor_meas.fB,
                        dt
                    ) : OrientationMeasurement{ avionics_gt.qIB },
            .Vinf = avionics.ADC.compute(mach_meas, T_meas),
            .pressure_alt_BE = avionics.ADC.compute(sensor_meas.P),
            .alt_BE_dot = sensor_hist ?
                            avionics.ADC.compute(
                            sensor_meas.P,
                            sensor_hist->P,
                            T_meas,
                            dt
                        ) : VerticalSpeedMeasurement{ avionics_gt.alt_BE_dot },
            .rho = avionics.ADC.compute(sensor_meas.P, T_meas)
        };

        hist = avionics_meas;

        return avionics_meas;
    }

    AvionicsGroundTruth build_avionics_gt(
        const dynamics::RigidBodyState& Xt,
        const dynamics::RigidBodyState& XEt,
        const aerodynamics::AerodynamicState& aero_t,
        const atmospheric::StaticAtmosphericState& atm_t,
        const geography::GeographicState& geo_t
    ) {

        atmospheric::MachNumber Mach = atmospheric::mps_to_mach(Xt.v, atm_t.T);
        atmospheric::StagnationAtmosphericState atm0_t = atmospheric::static_to_stagnation(atm_t, Mach);

        dynamics::TranslationalVelocity vE_BE{ XEt.q.data.conjugate() * XEt.v.data };
        double alt_BE_dot = vE_BE.data.dot(XEt.p.data.normalized());
        dynamics::Gravity gB = geography::gB(XEt.p, XEt.q);

        dynamics::EulerAngles eul;
        eul.set(Xt.q);

        AvionicsGroundTruth avionics_gt = {
            .gB = gB,
            .T = atm_t.T,
            .Mach = Mach,
            .qIB = Xt.q,
            .Vinf = aero_t.Vinf,
            .alt_BE = geo_t.alt,
            .alt_BE_dot = dynamics::VerticalSpeed{ alt_BE_dot },
            .rho = atm_t.rho,
        };

        return avionics_gt;
    }

    dynamics::RigidBodyState get_state_from_avionics(
        const sensors::SensorMeasurements& sensor_meas, 
        const AvionicsMeasurements& avionics_meas, 
        const Settings& avionics_settings
    ) {
        bool use_ins = avionics_settings.use_ins;

        dynamics::RigidBodyState Yt = {
            .p = use_ins ? avionics_meas.pI_BI_ins : sensor_meas.pI_BI_gnss,
            .v = use_ins ? avionics_meas.vB_BI_ins : sensor_meas.vB_BI_gnss,
            .q = avionics_meas.qIB,
            .w = sensor_meas.wB_BI
        };

        return Yt;
    }

}
