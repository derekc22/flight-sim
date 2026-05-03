#include <Eigen/Dense>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/control/control.hpp"
#include "simulation/linearization/linearization.hpp"
#include "simulation/trim/trim.hpp"
#include "simulation/constants/constants.hpp"

namespace estimation {

    RigidBodyStateEstimate step(
        const dynamics::RigidBodyState& zN_t, 
        const linearization::TrimLinearization& lin_sol, 
        const trim::TrimState<double>& trim_sol_state,
        const trim::TrimActuatorInputs<double>& trim_sol_input,
        const control::SurfaceActuatorInputs& u_surface_actual_prev, 
        const control::PropulsorActuatorInputs& u_propulsor_actual_prev
    ) {

        Eigen::VectorXd u_actual(constants::input_dim);
        u_actual << u_surface_actual_prev.elevator_cmd,
                    u_surface_actual_prev.aileron_cmd,
                    u_surface_actual_prev.rudder_cmd,
                    u_propulsor_actual_prev.front_propulsor_cmd,
                    u_propulsor_actual_prev.left_propulsor_cmd,
                    u_propulsor_actual_prev.right_propulsor_cmd;

        Eigen::VectorXd u_trim = trim::unpack_trim_control_inputs_T(trim_sol_input);
        Eigen::VectorXd ut_1 = control::unpack_actuator_inputs(u_surface_actual_prev, u_propulsor_actual_prev)
        Eigen::VectorXd ut_1_deviation = ut_1 - u_trim;

        Eigen::VectorXd z_trim = trim::unpack_trim_state_T(trim_sol_state);
        Eigen::VectorXd zt = trim::unpack_rigid_body_state(zN_t);
        Eigen::VectorXd zt_deviation = zt - z_trim;

        kalman_filter(zt_deviation, ut_1_deviation);
    }

}
