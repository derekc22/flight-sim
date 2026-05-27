#include "simulation/control/linear_quadratic/private.hpp"
#include "simulation/control/shared/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/control/linear_quadratic/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"

namespace control {

    ControlOutput make_control_output(const actuators::ActuatorInputsVector_T<double>& u_cmd) {
        actuators::SurfaceActuatorInputs_T<double> u_surface{};
        actuators::PropulsorActuatorInputs_T<double> u_propulsor{};

        u_surface.elevator_cmd = u_cmd[0];
        u_surface.aileron_cmd = u_cmd[1];
        u_surface.rudder_cmd = u_cmd[2];

        u_propulsor.front_propulsor_cmd = u_cmd[3];
        u_propulsor.left_propulsor_cmd = u_cmd[4];
        u_propulsor.right_propulsor_cmd = u_cmd[5];

        return { u_surface, u_propulsor };
    }

    dynamics::StateVector_T<double> unpack_linear_quadratic_controller_setpoint(const guidance::LinearFullStateFeedbackSetpoint& setpoint){
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
