#include "simulation/control/private/components/full_state/helpers.hpp"
#include "simulation/dynamics/public/data/helpers.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/guidance/public/data/types.hpp"

namespace control {

    dynamics::StateVector_T<double> unpack_state(const guidance::GuidanceSetpoint& setpoint) {
        dynamics::TranslationalVelocity vB_BI = setpoint.vB_BI;
        dynamics::AngularVelocity wB_BI = setpoint.wB_BI;
        dynamics::EulerAngles eulIB = setpoint.eulIB;

        dynamics::State_T<double> setpoint_packed {
            .vx = vB_BI.data(0),
            .vy = vB_BI.data(1),
            .vz = vB_BI.data(2),
            .p = wB_BI.p(),
            .q = wB_BI.q(),
            .r = wB_BI.r(),
            .phi = eulIB.phi(),
            .theta = eulIB.theta(),
        };
        return dynamics::unpack_state_T(setpoint_packed);
    }

}
