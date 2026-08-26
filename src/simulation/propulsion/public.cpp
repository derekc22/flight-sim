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
        // 2) autodiff/trim/linearization should have no side effects; they should only evaluate simulation state, not mutate it
        propulsor.propellers->prev_omega = propeller_omega.omega;
    }

    void commit_propeller_omega_state_set(actuators::PropulsorActuators& propulsor_actuators, const PropellerOmegaStateSet_T<double>& propeller_omega_state_set) {
        commit_propeller_omega_state(propulsor_actuators.front_propulsor, propeller_omega_state_set.front_propulsor);
        commit_propeller_omega_state(propulsor_actuators.left_propulsor, propeller_omega_state_set.left_propulsor);
        commit_propeller_omega_state(propulsor_actuators.right_propulsor, propeller_omega_state_set.right_propulsor);
    }

    dynamics::Wrench step_propulsive_forces_moments(actuators::PropulsorActuators& propulsor_actuators, const structural::CenterOfGravity& pB_GB, const dynamics::RigidBodyState& X, const atmospheric::StaticAtmosphericState& atm, const actuators::PropulsorActuatorInputs_T<double>& u, double dt) {
        dynamics::Twist_T<double> twist{
            .v = X.v.data,
            .w = X.w.data
        };

        const PropellerOmegaStateSet_T<double> propeller_omega_state_set{
            .front_propulsor = compute_propeller_omega_state_T<double>(propulsor_actuators.front_propulsor, u.front_propulsor_cmd, atm.rho, dt),
            .left_propulsor = compute_propeller_omega_state_T<double>(propulsor_actuators.left_propulsor, u.left_propulsor_cmd, atm.rho, dt),
            .right_propulsor = compute_propeller_omega_state_T<double>(propulsor_actuators.right_propulsor, u.right_propulsor_cmd, atm.rho, dt)
        };

        PropellerOmegaDotSet_T<double> propeller_omega_dot_set{
            .front_propulsor = propeller_omega_state_set.front_propulsor.omega_dot,
            .left_propulsor = propeller_omega_state_set.left_propulsor.omega_dot,
            .right_propulsor = propeller_omega_state_set.right_propulsor.omega_dot
        };

        dynamics::Wrench_T<double> wrench = step_propulsive_forces_moments_T<double>(propulsor_actuators, pB_GB.data, twist, atm, u, propeller_omega_dot_set);

        commit_propeller_omega_state_set(propulsor_actuators, propeller_omega_state_set);

        return { dynamics::Force{ wrench.F }, dynamics::Moment{ wrench.M } };
    }

}
