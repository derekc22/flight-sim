#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/propulsion/public.hpp"

namespace propulsion {

    void commit_propeller_omega_state(actuators::PropulsorActuator& propulsor, const PropellerOmegaState_T<double>& propeller_omega) {
        if (!propulsor.propellers.has_value()) {
            return;
        }

        // only the runtime/non-autodiff path should enter this branch because
        // 1) a type error will occur if this line attempts to assign a CppAD::AD<double> to a double field
        // 2) autodiff/trim/linearization/etc should have no side-effects; it should only evaluate simulation state, not mutate it 
        propulsor.propellers->prev_omega = propeller_omega.omega;
    }

    void commit_propellers_omega_state(actuators::PropulsorActuators& propulsor_actuators, const PropellerOmegaState_T<double>& front_propulsor_omega_state, const PropellerOmegaState_T<double>& left_propulsor_omega_state, const PropellerOmegaState_T<double>& right_propulsor_omega_state) {
        commit_propeller_omega_state(propulsor_actuators.front_propulsor, front_propulsor_omega_state);
        commit_propeller_omega_state(propulsor_actuators.left_propulsor, left_propulsor_omega_state);
        commit_propeller_omega_state(propulsor_actuators.right_propulsor, right_propulsor_omega_state);
    }

    dynamics::Wrench step_propulsive_forces_moments(actuators::PropulsorActuators& propulsor_actuators, const dynamics::RigidBodyState& X, const atmospheric::StaticAtmosphericState& atm, const actuators::PropulsorActuatorInputs_T<double>& u, double dt) {
        dynamics::Twist_T<double> twist{
            .v = X.v.data,
            .w = X.w.data
        };

        PropellerOmegaState_T<double> front_propulsor_omega_state = compute_propeller_omega_state_T<double>(
            propulsor_actuators.front_propulsor, 
            u.front_propulsor_cmd, 
            atm.rho, 
            dt
        );
        PropellerOmegaState_T<double> left_propulsor_omega_state = compute_propeller_omega_state_T<double>(
            propulsor_actuators.left_propulsor, 
            u.left_propulsor_cmd, 
            atm.rho, 
            dt
        );
        PropellerOmegaState_T<double> right_propulsor_omega_state = compute_propeller_omega_state_T<double>(
            propulsor_actuators.right_propulsor, 
            u.right_propulsor_cmd, 
            atm.rho, 
            dt
        );

        PropulsorOmegaDot_T<double> omega_dot{
            .front_propulsor = front_propulsor_omega_state.omega_dot,
            .left_propulsor = left_propulsor_omega_state.omega_dot,
            .right_propulsor = right_propulsor_omega_state.omega_dot
        };

        dynamics::Wrench_T<double> wrench = step_propulsive_forces_moments_T<double>(propulsor_actuators, twist, atm, actuators::PropulsorActuatorInputs_T<double>{ .front_propulsor_cmd = u.front_propulsor_cmd, .left_propulsor_cmd = u.left_propulsor_cmd, .right_propulsor_cmd = u.right_propulsor_cmd }, omega_dot);

        commit_propellers_omega_state(propulsor_actuators, front_propulsor_omega_state, left_propulsor_omega_state, right_propulsor_omega_state);

        return { dynamics::Force{ wrench.F }, dynamics::Moment{ wrench.M } };
    }

}
