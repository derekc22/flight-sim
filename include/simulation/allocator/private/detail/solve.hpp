#pragma once
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/control/public/data/types.hpp"
#include "simulation/qp/public/solver.hpp"

#include <array>

namespace allocator
{

	control::ControlOutput solve_qp_constrained(qp::Solver& solver,
		const constants::MatrixX_T<double, constants::input_dim, constants::input_dim>& hessian,
		const actuators::ActuatorInputsVector_T<double>& gradient,
		const actuators::ActuatorInputsVector_T<double>& u_0,
		const actuators::ActuatorInputsVector_T<double>& actuator_target,
		const actuators::ActuatorLimitsVector& limits,
		const std::array<bool, constants::input_dim>& actuator_mask);

	control::ControlOutput solve_qp_unconstrained(
		const constants::MatrixX_T<double, constants::input_dim, constants::input_dim>& hessian,
		const actuators::ActuatorInputsVector_T<double>& gradient,
		const actuators::ActuatorInputsVector_T<double>& u_0,
		const actuators::ActuatorInputsVector_T<double>& actuator_target,
		const actuators::ActuatorLimitsVector& limits,
		const std::array<bool, constants::input_dim>& actuator_mask);

} // namespace allocator
