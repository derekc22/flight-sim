#include "simulation/control/linear_quadratic/controllers/lqr.hpp"

namespace control {

    LinearQuadraticRegulator::LinearQuadraticRegulator(const LinearQuadraticRegulatorParameters& params) :
        params(params), policy(params)
    {};

    trim::TrimStateVector_T<double> LinearQuadraticRegulator::unpack_linear_quadratic_regulator_setpoint(const guidance::LinearFullStateFeedbackSetpoint& setpoint){
        dynamics::LinearVelocity vB_BI = setpoint.vB_BI;
        dynamics::AngularVelocity wB_BI = setpoint.wB_BI;
        dynamics::EulerAngles eulIB = setpoint.eulIB;

        trim::TrimState<double> setpoint_packed { 
            .vx = vB_BI.data(0),
            .vy = vB_BI.data(1),
            .vz = vB_BI.data(2),
            .p = wB_BI.p(),
            .q = wB_BI.q(),
            .r = wB_BI.r(),
            .phi = eulIB.phi(),
            .theta = eulIB.theta(),
        };
        return trim::unpack_trim_state_T(setpoint_packed);
    }

    LinearQuadraticControllerInput LinearQuadraticRegulator::make_linear_quadratic_controller_input(const LinearFullStateFeedbackControllerInput& controller_input){
        dynamics::RigidBodyState zN_t = controller_input.zN_t;
        guidance::LinearFullStateFeedbackSetpoint setpoint = controller_input.setpoint;

        return {
            .zN_t = trim::unpack_rigid_body_state(controller_input.zN_t),
            .zN_t_des = unpack_linear_quadratic_regulator_setpoint(setpoint),
            .A = controller_input.A,
            .B = controller_input.B
        };
    }

    ControlOutput LinearQuadraticRegulator::step(const LinearFullStateFeedbackControllerInput& controller_input){
        SurfaceActuatorInputs u_surface{};
        PropulsorActuatorInputs u_propulsor{};

        trim::TrimActuatorInputsVector_T<double> u_deviation = policy.step(
            make_linear_quadratic_controller_input(controller_input)
        );
        trim::TrimActuatorInputsVector_T<double> u_trim = trim::unpack_trim_actuator_inputs_T(controller_input.u_sol_trim);

        trim::TrimActuatorInputsVector_T<double> u_cmd = u_deviation + u_trim;

        u_surface.elevator_cmd = u_cmd[0];
        u_surface.aileron_cmd = u_cmd[1];
        u_surface.rudder_cmd = u_cmd[2];

        u_propulsor.front_propulsor_cmd = u_cmd[3];
        u_propulsor.left_propulsor_cmd = u_cmd[4];
        u_propulsor.right_propulsor_cmd = u_cmd[5];

        return { u_surface, u_propulsor };
    }

}
