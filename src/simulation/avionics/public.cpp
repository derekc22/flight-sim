#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/avionics/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"

namespace avionics {

    MeasurementCache AvionicsProperties::step(const MeasurementGroundTruth& meas_gt) {
        SensorMeasurements sensor_meas {
            .alpha = sensors.aoa_vane.measure(meas_gt.alpha),
            .fB = sensors.accelerometer.measure(meas_gt.fB),
            .wB_BI = sensors.gyro.measure(meas_gt.wB_BI),
            .P0 = sensors.pitot_tube.measure(meas_gt.P0),
            .P = sensors.static_port.measure(meas_gt.P),
            .T0 = sensors.tat_probe.measure(meas_gt.T0),
            .pI_BI_gnss = sensors.gnss.measure(meas_gt.pI_BI),
            .vB_BI_gnss = sensors.gnss.measure(meas_gt.vB_BI),
            .heading_BE = sensors.magnetometer.measure(meas_gt.heading)
        };

        MachNumberMeasurement curr_Mach_meas{ atmospheric::compute_mach(sensor_meas.P0, sensor_meas.P).data };
        StaticAirTemperatureMeasurement curr_T_meas{ atmospheric::T_from_T0(sensor_meas.T0, curr_Mach_meas) };

        ComputerMeasurements computer_meas {
            .pI_BI_ins = hist.computers ?
                         computers.INS.compute(
                            hist.computers->pI_BI_ins,
                            hist.computers->vB_BI_ins,
                            sensor_meas.fB,
                            meas_gt.gB,
                            hist.computers->qIB
                        ) : PositionMeasurement{ meas_gt.pI_BI.data },
            .vB_BI_ins = hist.computers ?
                         computers.INS.compute(
                            hist.computers->vB_BI_ins,
                            sensor_meas.fB,
                            meas_gt.gB,
                            sensor_meas.wB_BI
                        ) : TranslationalVelocityMeasurement{ meas_gt.vB_BI.data },
            .T = curr_T_meas,
            .Mach = curr_Mach_meas,
            .Vinf = computers.ADC.compute(curr_Mach_meas, curr_T_meas),
            .pressure_alt_BE = computers.ADC.compute(sensor_meas.P),
            .alt_BE_dot = hist.sensors ?
                          computers.ADC.compute(
                            sensor_meas.P,
                            hist.sensors->P,
                            curr_T_meas
                        ) : VerticalSpeedMeasurement{ meas_gt.alt_BE_dot.data },
            .rho = computers.ADC.compute(sensor_meas.P, curr_T_meas),
            .qIB = hist.computers ?
                   computers.AHRS.compute(
                      hist.computers->qIB,
                      sensor_meas.wB_BI
                    ) : OrientationMeasurement{ meas_gt.qIB.data }
        };

        cache = MeasurementCache{ .sensors = sensor_meas, .computers = computer_meas };
        hist = MeasurementHistory{ .sensors = cache.sensors, .computers = cache.computers };

        return cache;
    }

    MeasurementGroundTruth build_measurement_gt(
        const dynamics::RigidBodyState& Xt,
        const dynamics::RigidBodyState& XEt,
        const aerodynamics::AerodynamicState& aero_t,
        const atmospheric::StaticAtmosphericState& atm_t,
        const geography::GeographicState& geo_t,
        const dynamics::Mass& mass,
        const atmospheric::Wind & wind,
        const dynamics::Wrench& WB_net
    ) {

        atmospheric::MachNumber Mach = atmospheric::mps_to_mach(Xt.v, atm_t.T);
        atmospheric::StagnationAtmosphericState atm0_t = atmospheric::static_to_stagnation(atm_t, Mach);

        dynamics::TranslationalVelocity vE_BE{ XEt.q.data.conjugate() * XEt.v.data };
        double alt_BE_dot = vE_BE.data.dot(XEt.p.data.normalized());
        dynamics::Gravity gB = geography::gB(XEt.p, XEt.q);

        dynamics::EulerAngles eul;
        eul.set(Xt.q);

        avionics::MeasurementGroundTruth meas_gt = {
            .alpha = aero_t.alpha,
            .fB = dynamics::TranslationalAcceleration{ WB_net.F.data / mass.data - gB.data },
            .gB = gB,
            .wB_BI = Xt.w,
            .P0 = atm0_t.P0,
            .P = atm_t.P,
            .T0 = atm0_t.T0,
            .pI_BI = Xt.p,
            .vB_BI = Xt.v,

            .T = atm_t.T,
            .Mach = Mach,
            .heading = geography::Heading{ eul.psi() },
            .qIB = Xt.q,
            .Vinf = aero_t.Vinf,
            .alt_BE = geo_t.alt,
            .alt_BE_dot = dynamics::VerticalSpeed{ alt_BE_dot },
            .rho = atm_t.rho,
        };

        return meas_gt;
    }

    dynamics::RigidBodyState get_state_from_avionics(const MeasurementCache& cache, const Settings& avionics_settings) {
        bool use_gnss = avionics_settings.use_gnss;

        dynamics::RigidBodyState Yt = {
            .p = use_gnss ? cache.sensors.pI_BI_gnss : cache.computers.pI_BI_ins,
            .v = use_gnss ? cache.sensors.vB_BI_gnss : cache.computers.vB_BI_ins,
            .q = cache.computers.qIB,
            .w = cache.sensors.wB_BI
        };

        return Yt;
    }

}
