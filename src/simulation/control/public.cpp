#include <array>
#include "simulation/control/public.hpp"
#include "simulation/control/shared/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/util/public.hpp"

namespace control {

    VirtualControlOutputSet ControlProperties::step(const ControllerInputs& inputs, double dt) {
        VirtualControlOutput_T<double> out{};
        std::array<bool, constants::virtual_input_dim> active_mask{};

        if (attitude_controller && inputs.attitude_controller_input.has_value()) {
            VirtualControlOutput_T<double> _out = attitude_controller(inputs.attitude_controller_input.value(), dt);
            out.F += _out.F;
            out.M += _out.M;
        	util::fill_arr(active_mask, 3, 6, true);

        }
        if (velocity_controller && inputs.velocity_controller_input.has_value()) {
            VirtualControlOutput_T<double> _out = velocity_controller(inputs.velocity_controller_input.value(), dt);
            out.F += _out.F;
            out.M += _out.M;
        	util::fill_arr(active_mask, 0, 1, true);
        }
        if (linear_quadratic_controller && inputs.linear_quadratic_controller_input.has_value()) {
            VirtualControlOutput_T<double> _out = linear_quadratic_controller(inputs.linear_quadratic_controller_input.value(), dt);
            out.F += _out.F;
            out.M += _out.M;
        	util::fill_arr(active_mask, 0, 6, true);
        }
        if (nonlinear_controller && inputs.nonlinear_controller_input.has_value()) {
            VirtualControlOutput_T<double> _out = nonlinear_controller(inputs.nonlinear_controller_input.value(), dt);
            out.F += _out.F;
            out.M += _out.M;
        	util::fill_arr(active_mask, 0, 6, true);
        }

        return {
	        {
		        { out.F },
	        	{ out.M }
	        },
        	active_mask
        };
    }

    ControllerInputs ControlProperties::build_controller_inputs(
        const dynamics::RigidBodyState& Zt, 
        const trim::TrimSolution& trim_sol, 
        const linearization::VirtualLocalLinearization& virtual_lin_sol, 
        const actuators::SurfaceActuators& surface_actuators, 
        const actuators::PropulsorActuators& propulsor_actuators, 
        const guidance::GuidanceSetpoint& setpoint,
        const dynamics::WrenchVector_T<double> delta_mu_vec_t_1
    ) {
        ControllerInputs controller_inputs;

        if (attitude_controller_type != ControllerType::None) {
            controller_inputs.attitude_controller_input.emplace(
				AttitudeControllerInput{
			    	.Zt = Zt,
			    	.setpoint = guidance::AttitudeSetpoint{ setpoint },
			    	.delta_mu_vec_t_1 = delta_mu_vec_t_1
			    }
            );
        }
        if (velocity_controller_type != ControllerType::None) {
            controller_inputs.velocity_controller_input.emplace(
                VelocityControllerInput{
                    .Zt = Zt,
                    .setpoint = guidance::VelocitySetpoint{ setpoint },
                    .delta_mu_vec_t_1 = delta_mu_vec_t_1
                }
            );
        }
        if (linear_quadratic_controller_type != ControllerType::None) {
            controller_inputs.linear_quadratic_controller_input.emplace(
                LinearQuadraticControllerInput{
				    .Zt = Zt,
				    .mu_sol_trim = trim_sol.wrench,
				    .surface_actuators = surface_actuators,
				    .propulsor_actuators = propulsor_actuators,
				    .virtual_linearization = virtual_lin_sol,
				    .setpoint = guidance::LinearQuadraticSetpoint{ setpoint },
				    .delta_mu_vec_t_1 = delta_mu_vec_t_1
			    }
            );
        }
        if (nonlinear_controller_type != ControllerType::None) {
            controller_inputs.nonlinear_controller_input.emplace(
                NonlinearControllerInput{
				    .Zt = Zt,
				    .surface_actuators = surface_actuators,
				    .propulsor_actuators = propulsor_actuators,
				    .setpoint = guidance::NonlinearSetpoint{ setpoint }
			    }
            );
        }

		return controller_inputs;
	}
}
