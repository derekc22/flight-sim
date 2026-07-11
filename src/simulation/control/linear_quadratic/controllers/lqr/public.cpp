#include "simulation/actuators/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqr/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/control/linear_quadratic/private.hpp"

namespace control {

    LinearQuadraticRegulator::LinearQuadraticRegulator(const LinearQuadraticRegulatorParameters& params) :
        params(params), policy(params)
    {};

    LinearQuadraticPolicyInput LinearQuadraticRegulator::make_linear_quadratic_policy_input(const LinearQuadraticControllerInput& input) {
        dynamics::RigidBodyState Zt = input.Zt;
        guidance::LinearQuadraticSetpoint setpoint = input.setpoint;

        dynamics::StateVector zt_deviation = dynamics::unpack_state(Zt) - unpack_state(setpoint);

        return {
            .zt = zt_deviation,
            .A = input.A,
            .B = input.B
        };
    }

    ControlOutput LinearQuadraticRegulator::step(const LinearQuadraticControllerInput& input, double dt) {
        actuators::ActuatorInputsVector u_deviation = policy.step(
            make_linear_quadratic_policy_input(input)
        );
        actuators::ActuatorInputsVector u_trim = actuators::unpack_actuator_inputs(input.u_sol_trim);
        actuators::ActuatorInputsVector u_cmd = u_deviation + u_trim;

        return make_control_output(u_cmd);
    }

}
