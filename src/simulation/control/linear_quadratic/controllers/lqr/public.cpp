#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqr/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/control/linear_quadratic/private.hpp"

namespace control {

    LinearQuadraticRegulator::LinearQuadraticRegulator(const LinearQuadraticRegulatorParameters& params) :
        params(params), policy(params)
    {};

    LinearQuadraticControllerInput LinearQuadraticRegulator::make_linear_quadratic_controller_input(const LinearFullStateFeedbackControllerInput& controller_input){
        dynamics::RigidBodyState zN_t = controller_input.zN_t;
        guidance::LinearFullStateFeedbackSetpoint setpoint = controller_input.setpoint;

        return {
            .zN_t = dynamics::unpack_rigid_body_state(controller_input.zN_t),
            .zN_t_des = unpack_linear_quadratic_controller_setpoint(setpoint),
            .A = controller_input.A,
            .B = controller_input.B
        };
    }

    ControlOutput LinearQuadraticRegulator::step(const LinearFullStateFeedbackControllerInput& controller_input){
        actuators::ActuatorInputsVector_T<double> u_deviation = policy.step(
            make_linear_quadratic_controller_input(controller_input)
        );
        actuators::ActuatorInputsVector_T<double> u_trim = actuators::unpack_actuator_inputs_T(controller_input.u_sol_trim);
        actuators::ActuatorInputsVector_T<double> u_cmd = u_deviation + u_trim;

        return make_control_output(u_cmd);
    }

}
