#include "simulation/control/linear_quadratic/controllers/lqr.hpp"

namespace control {

    LinearQuadraticRegulator::LinearQuadraticRegulator(const LinearQuadraticRegulatorParameters& params) :
        policy( LinearQuadraticControlLaw({ 
                .Q = params.Q, 
                .R = params.R
            })
        )
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

    LinearQuadraticControlLawInput LinearQuadraticRegulator::make_linear_quadratic_control_law_input(
        const LinearQuadraticRegulatorInput& ctrl_law_input
    ){
        dynamics::RigidBodyState zN_t = ctrl_law_input.zN_t;
        guidance::LinearFullStateFeedbackSetpoint setpoint = ctrl_law_input.setpoint;

        return {
            .meas = trim::unpack_rigid_body_state(ctrl_law_input.zN_t),
            .meas_des = unpack_linear_quadratic_regulator_setpoint(setpoint),
            .A = ctrl_law_input.A,
            .B = ctrl_law_input.B
        };
    }

    ControlOutput LinearQuadraticRegulator::step(const LinearQuadraticRegulatorInput& ctrl_law_input){
        SurfaceActuatorInputs u_surface{};
        PropulsorActuatorInputs u_propulsor{};

        Eigen::VectorXd u_deviation = policy.step(
            make_linear_quadratic_control_law_input(ctrl_law_input)
        );
        Eigen::VectorXd u_trim = trim::unpack_trim_actuator_inputs_T(ctrl_law_input.u_sol_trim);

        Eigen::VectorXd u_cmd = u_deviation + u_trim;

        u_surface.elevator_cmd = u_cmd[0];
        u_surface.aileron_cmd = u_cmd[1];
        u_surface.rudder_cmd = u_cmd[2];

        u_propulsor.front_propulsor_cmd = u_cmd[3];
        u_propulsor.left_propulsor_cmd = u_cmd[4];
        u_propulsor.right_propulsor_cmd = u_cmd[5];

        return { u_surface, u_propulsor };
    }

}
