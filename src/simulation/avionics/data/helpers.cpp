#include "simulation/atmospheric/public/detail/compressible.hpp"
#include "simulation/avionics/public/data/helpers.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/geography/public/detail/gravity.hpp"

namespace avionics {

    AvionicsGroundTruth build_avionics_gt(
        const dynamics::RigidBodyState& Xt,
        const dynamics::RigidBodyState& XEt,
        const aerodynamics::AerodynamicState& aero_t,
        const atmospheric::StaticAtmosphericState& atm_t,
        const geography::GeographicState& geo_t
    ) {

        atmospheric::MachNumber Mach = atmospheric::mps_to_mach(Xt.v, atm_t.T);
        dynamics::TranslationalVelocity vE_BE{ XEt.q.data.conjugate() * XEt.v.data };
        dynamics::VerticalSpeed alt_BE_dot{ vE_BE.data.dot(XEt.p.data.normalized()) };
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
            .alt_BE_dot = alt_BE_dot,
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
