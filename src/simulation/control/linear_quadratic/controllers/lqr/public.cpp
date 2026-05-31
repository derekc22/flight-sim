#include "simulation/actuators/public.hpp"
#include "simulation/control/linear_quadratic/controllers/lqr/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/control/linear_quadratic/private.hpp"

namespace control {

    LinearQuadraticRegulator::LinearQuadraticRegulator(const LinearQuadraticRegulatorParameters& params) :
        params(params), policy(params)
    {};

    LinearQuadraticPolicyInput LinearQuadraticRegulator::make_linear_quadratic_policy_input(const LinearQuadraticControllerInput& input){
        dynamics::RigidBodyState Zt = input.Zt;
        guidance::LinearQuadraticSetpoint setpoint = input.setpoint;

        dynamics::StateVector_T<double> zt_vec_deviation = dynamics::unpack_state(Zt) - unpack_state(setpoint);

        return {
            .zt_vec = zt_vec_deviation,
            .A = input.A,
            .B = input.B
        };
    }

    ControlOutput LinearQuadraticRegulator::step(const LinearQuadraticControllerInput& input){
        actuators::ActuatorInputsVector_T<double> u_deviation = policy.step(
            make_linear_quadratic_policy_input(input)
        );
        actuators::ActuatorInputsVector_T<double> u_trim = actuators::unpack_actuator_inputs_T(input.u_sol_trim);
        actuators::ActuatorInputsVector_T<double> u_cmd = u_deviation + u_trim;

        return make_control_output(u_cmd);
    }

}
