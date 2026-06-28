#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/sensors/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"

namespace sensors {

    SensorMeasurements SensorProperties::step(const SensorGroundTruth& sensor_gt) {
        SensorMeasurements sensor_meas {
            .alpha = sensors.aoa_vane.measure(sensor_gt.alpha),
            .fB = sensors.accelerometer.measure(sensor_gt.fB),
            .wB_BI = sensors.gyro.measure(sensor_gt.wB_BI),
            .P0 = sensors.pitot_tube.measure(sensor_gt.P0),
            .P = sensors.static_port.measure(sensor_gt.P),
            .T0 = sensors.tat_probe.measure(sensor_gt.T0),
            .pI_BI_gnss = sensors.gnss.measure(sensor_gt.pI_BI),
            .vB_BI_gnss = sensors.gnss.measure(sensor_gt.vB_BI),
            .heading_BE = sensors.magnetometer.measure(sensor_gt.heading)
        };

        hist = sensor_meas;

        return sensor_meas;
    }

    SensorGroundTruth build_sensor_gt(
        const dynamics::RigidBodyState& Xt,
        const dynamics::RigidBodyState& XEt,
        const aerodynamics::AerodynamicState& aero_t,
        const atmospheric::StaticAtmosphericState& atm_t,
        const dynamics::Mass& mass,
        const dynamics::Wrench& WB_net
    ) {

        atmospheric::MachNumber Mach = atmospheric::mps_to_mach(Xt.v, atm_t.T);
        atmospheric::StagnationAtmosphericState atm0_t = atmospheric::static_to_stagnation(atm_t, Mach);
        dynamics::Gravity gB = geography::gB(XEt.p, XEt.q);
        dynamics::EulerAngles eul;
        eul.set(Xt.q);

        SensorGroundTruth sensor_gt = {
            .alpha = aero_t.alpha,
            .fB = dynamics::TranslationalAcceleration{ WB_net.F.data / mass.data - gB.data },
            .wB_BI = Xt.w,
            .P0 = atm0_t.P0,
            .P = atm_t.P,
            .T0 = atm0_t.T0,
            .pI_BI = Xt.p,
            .vB_BI = Xt.v,
            .heading = geography::Heading{ eul.psi() },
        };

        return sensor_gt;
    }

}
