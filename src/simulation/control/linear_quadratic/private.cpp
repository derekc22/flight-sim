#include "simulation/control/linear_quadratic/private.hpp"
#include "simulation/control/shared/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/control/linear_quadratic/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"

namespace control {

    ControlOutput make_control_output(const actuators::ActuatorInputsVector& u_cmd) {
        return actuators::pack_actuator_inputs(u_cmd);
    }

    dynamics::StateVector unpack_state(const guidance::LinearQuadraticSetpoint& setpoint) {
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
        return dynamics::unpack_state(setpoint_packed);
    }

}
