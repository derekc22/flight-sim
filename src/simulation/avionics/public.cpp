#include <optional>
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/avionics/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/sensors/public.hpp"

namespace avionics {

    AvionicsMeasurements AvionicsProperties::step(
        const sensors::SensorMeasurements& sensor_meas,
        const std::optional<sensors::SensorMeasurements> sensor_hist, // will always be populated by the time AvionicsProperties::step is called, but std::optional is kept here for consistency
        const sensors::SensorGroundTruth& sensor_gt,
        const AvionicsGroundTruth& avionics_gt
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
                            hist->qIB
                        ) : sensors::PositionMeasurement{ sensor_gt.pI_BI },
            .vB_BI_ins = hist ?
                         avionics.INS.compute(
                            hist->vB_BI_ins,
                            sensor_meas.fB,
                            avionics_gt.gB,
                            sensor_meas.wB_BI
                        ) : sensors::TranslationalVelocityMeasurement{ sensor_gt.vB_BI },
            .T = T_meas,
            .Mach = mach_meas,
            .qIB = hist ?
                   avionics.AHRS.compute(
                      hist->qIB,
                      sensor_meas.wB_BI,
                      sensor_meas.fB
                    ) : OrientationMeasurement{ avionics_gt.qIB },
            .Vinf = avionics.ADC.compute(mach_meas, T_meas),
            .pressure_alt_BE = avionics.ADC.compute(sensor_meas.P),
            .alt_BE_dot = sensor_hist ?
                          avionics.ADC.compute(
                            sensor_meas.P,
                            sensor_hist->P,
                            T_meas
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
