#include <Eigen/Dense>
#include <stdexcept>
#include "simulation/control/public.hpp"
#include "simulation/control/shared/public.hpp"

namespace control {

    VirtualControlOutput ControlProperties::step(const ControllerInputs& inputs, double dt, bool trim_flag) {
        VirtualControlOutput_T<double> out{};

        if (!trim_flag) {
            if (attitude_controller) {
                VirtualControlOutput_T<double> _out = attitude_controller(inputs.attitude_controller_input, dt);
                out.F += _out.F;
                out.M += _out.M;
            }
            if (velocity_controller) {
                VirtualControlOutput_T<double> _out = velocity_controller(inputs.velocity_controller_input, dt);
                out.F += _out.F;
                out.M += _out.M;
            }
            if (nonlinear_controller) {
                VirtualControlOutput_T<double> _out = nonlinear_controller(inputs.nonlinear_controller_input, dt);
                out.F += _out.F;
                out.M += _out.M;
            }
            if (linear_quadratic_controller) { 
                throw std::runtime_error("control::ControlProperties::step LinearQuadraticController requires trim"); 
            }
        }

        if (trim_flag) {
            if (linear_quadratic_controller) {
                VirtualControlOutput_T<double> _out = linear_quadratic_controller(inputs.linear_quadratic_controller_input, dt);
                out.F += _out.F;
                out.M += _out.M;
            }
        }

        return { { out.F }, { out.M } };
    }

	ControllerInputs build_controller_inputs(const dynamics::RigidBodyState& Zt, const trim::TrimSolution& trim_sol, const linearization::VirtualLocalLinearization& virtual_lin_sol, const actuators::SurfaceActuators& surface_actuators, const actuators::PropulsorActuators& propulsor_actuators, const guidance::GuidanceSetpoint& setpoint) {
		ControllerInputs controller_inputs {
			.attitude_controller_input = AttitudeControllerInput{
				.Zt = Zt,
				.surface_actuators = surface_actuators,
				.setpoint = guidance::AttitudeSetpoint{ setpoint }
			},
			.velocity_controller_input = VelocityControllerInput{
				.Zt = Zt,
				.propulsor_actuators = propulsor_actuators,
				.setpoint = guidance::VelocitySetpoint{ setpoint }
			},
			.linear_quadratic_controller_input = LinearQuadraticControllerInput{
				.Zt = Zt,
				.mu_sol_trim = trim_sol.wrench,
				.surface_actuators = surface_actuators,
				.propulsor_actuators = propulsor_actuators,
				.virtual_linearization = virtual_lin_sol,
				.setpoint = guidance::LinearQuadraticSetpoint{ setpoint }
			},
			.nonlinear_controller_input = NonlinearControllerInput{
				.Zt = Zt,
				.surface_actuators = surface_actuators,
				.propulsor_actuators = propulsor_actuators,
				.setpoint = guidance::NonlinearSetpoint{ setpoint }
			}
		};

		return controller_inputs;
	}
}
