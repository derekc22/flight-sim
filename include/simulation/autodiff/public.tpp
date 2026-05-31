#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/propulsion/public.hpp"
#include "simulation/util/public.hpp"

namespace autodiff {

    template <typename T>
    actuators::SurfaceActuatorInputs_T<T> pack_surface_actuator_inputs_T(const actuators::ActuatorInputs_T<T>& u, const actuators::FixedActuatorInputs& fixed_actuator_inputs) {
        return {
            .elevator_cmd = u.elevator_cmd,
            .aileron_cmd = u.aileron_cmd,
            .rudder_cmd = u.rudder_cmd,
            .flap_cmd = T(fixed_actuator_inputs.flap),
            .spoiler_cmd = T(fixed_actuator_inputs.spoiler),
        };
    }

    template <typename T>
    actuators::PropulsorActuatorInputs_T<T> pack_propulsor_actuator_inputs_T(const actuators::ActuatorInputs_T<T>& u) {
        return {
            .front_propulsor_cmd = u.front_propulsor_cmd,
            .left_propulsor_cmd = u.left_propulsor_cmd,
            .right_propulsor_cmd = u.right_propulsor_cmd
        };
    }

    template <typename T>
    dynamics::Wrench_T<T> compute_net_wrench_T(const dynamics::State_T<T>& x, const dynamics::Twist_T<T>& twist, const actuators::ActuatorInputs_T<T>& u, const AutoDiffModel& model, const operating::OperatingConditions& conditions) {
        const actuators::SurfaceActuatorInputs_T<T> surface_actuator_inputs = pack_surface_actuator_inputs_T(u, model.fixed_actuator_inputs);
        const dynamics::Wrench_T<T> aero_wrench = aerodynamics::step_aero_forces_moments_T<T>(model.aerodynamic, model.structural, twist, conditions.static_atm_state, surface_actuator_inputs, conditions.windB);

        const actuators::PropulsorActuatorInputs_T<T> propulsor_actuator_inputs = pack_propulsor_actuator_inputs_T(u);
        const dynamics::Wrench_T<T> prop_wrench = propulsion::step_propulsive_forces_moments_T<T>(model.propulsor_actuators, twist, conditions.static_atm_state, propulsor_actuator_inputs, propulsion::PropulsorOmegaDot_T<T>{});

        return {
            .F = aero_wrench.F + prop_wrench.F + T(model.structural.mass.data) * dynamics::gB_T(x.phi, x.theta),
            .M = aero_wrench.M + prop_wrench.M,
        };
    }

    template <typename T>
    dynamics::StateDot_T<T> compute_state_dot_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u, const AutoDiffModel& model, const operating::OperatingConditions& conditions) {
        const dynamics::Twist_T<T> twist = dynamics::build_twist_from_state_T(x);
        const dynamics::Wrench_T<T> net_wrench = compute_net_wrench_T<T>(x, twist, u, model, conditions);
        const constants::Vector3_T<T> v_dot = dynamics::ddtB_vB_BI_T<T>(twist.v, twist.w, model.structural.mass.data, net_wrench.F);
        const constants::Vector3_T<T> w_dot = dynamics::ddtB_wB_BI_T<T>(twist.w, model.structural.J.data, net_wrench.M);
        const constants::Vector3_T<T> eul_dot = dynamics::wB_BI_to_eul_dot_T<T>(twist.w, x.theta, x.phi);

        return {
            .vx_dot = v_dot.x(),
            .vy_dot = v_dot.y(),
            .vz_dot = v_dot.z(),
            .p_dot = w_dot.x(),
            .q_dot = w_dot.y(),
            .r_dot = w_dot.z(),
            .phi_dot = eul_dot.x(),
            .theta_dot = eul_dot.y(),
        };
    }

}
