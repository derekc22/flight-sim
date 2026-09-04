#include "simulation/allocator/private/detail/effectiveness.hpp"

#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/autodiff/public/detail/cppad.hpp"
#include "simulation/autodiff/public/detail/evaluation.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/dynamics/public/data/helpers.hpp"
#include "simulation/operating/public/data/helpers.hpp"

#include <tuple>

namespace allocator
{

	std::tuple<EffectivenessMatrix, dynamics::WrenchVector_T<double>> compute_effectiveness_matrix(
		const autodiff::AutoDiffModel& model,
		const operating::OperatingPoint_T<double>& operating_point,
		const operating::OperatingConditions& conditions)
	{

		const operating::StateInputVector_T<double> xu =
			operating::unpack_state_input_T<double>(operating_point.state, operating_point.input);
		const auto [x, u] = operating::split_state_input_vector_T(xu);

		const dynamics::StateVector_T<CppAD::AD<double>> x_ad = autodiff::double_to_cppad_double(x);

		CppAD::eigen_vector<CppAD::AD<double>> u_tracked =
			autodiff::start_autodiff_tracking(u); // start of autodiff tracking

		const actuators::ActuatorInputsVector_T<CppAD::AD<double>> u_eigen =
			autodiff::eigen_vector_from_cppad_vector<CppAD::AD<double>, constants::input_dim>(u_tracked);

		const dynamics::State_T<CppAD::AD<double>> xt = dynamics::pack_state_T(x_ad);
		const actuators::ActuatorInputs_T<CppAD::AD<double>> ut = actuators::pack_actuator_inputs_T(u_eigen);
		const operating::OperatingPoint_T<CppAD::AD<double>> operating_point_cppad =
			operating::pack_state_input_T(xt, ut);

		const dynamics::Wrench_T<CppAD::AD<double>> W_net =
			autodiff::compute_net_wrench_T<CppAD::AD<double>>(operating_point_cppad, model, conditions, constants::dt);
		const dynamics::WrenchVector_T<CppAD::AD<double>> W_net_vec = dynamics::unpack_wrench_T(W_net);

		const CppAD::eigen_vector<CppAD::AD<double>> W_net_cppad = autodiff::cppad_vector_from_eigen_vector(W_net_vec);

		CppAD::ADFun<double> f(u_tracked, W_net_cppad); // end of autodiff tracking

		const constants::MatrixX_T<double, constants::virtual_input_dim, constants::input_dim> jac_full =
			autodiff::compute_jac<constants::virtual_input_dim, constants::input_dim>(f, u);

		// evaluate W_net_vec
		const dynamics::WrenchVector_T<double> W_net_vec_double =
			autodiff::evaluate_tracked_vector<constants::virtual_input_dim, constants::input_dim>(f, u);

		return {jac_full, W_net_vec_double};
	}

} // namespace allocator
