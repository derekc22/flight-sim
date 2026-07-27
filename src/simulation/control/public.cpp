#include <Eigen/Dense>
#include "simulation/control/public.hpp"
#include "simulation/control/shared/public.hpp"

namespace control {

    VirtualControlOutput ControlProperties::step(const ControllerInputs& inputs, double dt) {
        VirtualControlOutput_T<double> out{};

        if (attitude_controller && inputs.attitude_controller_input.has_value()) {
            VirtualControlOutput_T<double> _out = attitude_controller(inputs.attitude_controller_input.value(), dt);
            out.F += _out.F;
            out.M += _out.M;
        }
        if (velocity_controller && inputs.velocity_controller_input.has_value()) {
            VirtualControlOutput_T<double> _out = velocity_controller(inputs.velocity_controller_input.value(), dt);
            out.F += _out.F;
            out.M += _out.M;
        }
        if (linear_quadratic_controller && inputs.linear_quadratic_controller_input.has_value()) {
            VirtualControlOutput_T<double> _out = linear_quadratic_controller(inputs.linear_quadratic_controller_input.value(), dt);
            out.F += _out.F;
            out.M += _out.M;
        }
        if (nonlinear_controller && inputs.nonlinear_controller_input.has_value()) {
            VirtualControlOutput_T<double> _out = nonlinear_controller(inputs.nonlinear_controller_input.value(), dt);
            out.F += _out.F;
            out.M += _out.M;
        }

        return { { out.F }, { out.M } };
    }

    ControllerInputs ControlProperties::build_controller_inputs(
        const dynamics::RigidBodyState& Zt, 
        const trim::TrimSolution& trim_sol, 
        const linearization::VirtualLocalLinearization& virtual_lin_sol, 
        const actuators::SurfaceActuators& surface_actuators, 
        const actuators::PropulsorActuators& propulsor_actuators, 
        const guidance::GuidanceSetpoint& setpoint
    ) {
        ControllerInputs controller_inputs;

        if (attitude_controller_type != ControllerType::None) {
            controller_inputs.attitude_controller_input.emplace(AttitudeControllerInput{
				.Zt = Zt,
				.setpoint = guidance::AttitudeSetpoint{ setpoint }
			});
        }
        if (velocity_controller_type != ControllerType::None) {
            controller_inputs.velocity_controller_input.emplace(VelocityControllerInput{
				.Zt = Zt,
				.setpoint = guidance::VelocitySetpoint{ setpoint }
			});
        }
        if (linear_quadratic_controller_type != ControllerType::None) {
            controller_inputs.linear_quadratic_controller_input.emplace(LinearQuadraticControllerInput{
				.Zt = Zt,
				.mu_sol_trim = trim_sol.wrench,
				.surface_actuators = surface_actuators,
				.propulsor_actuators = propulsor_actuators,
				.virtual_linearization = virtual_lin_sol,
				.setpoint = guidance::LinearQuadraticSetpoint{ setpoint }
			});
        }
        if (nonlinear_controller_type != ControllerType::None) {
            controller_inputs.nonlinear_controller_input.emplace(NonlinearControllerInput{
				.Zt = Zt,
				.surface_actuators = surface_actuators,
				.propulsor_actuators = propulsor_actuators,
				.setpoint = guidance::NonlinearSetpoint{ setpoint }
			});
        }

		return controller_inputs;
	}
}
