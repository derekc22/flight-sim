#include <tuple>
#include "simulation/allocator/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/control/public.hpp"
#include "simulation/autodiff/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/operating/public.hpp"

namespace allocator {

    control::ControlOutput AllocatorProperties::step(const AllocatorInput& input) {

        auto [E, mu_0] = compute_effectiveness_matrix(input.model, input.operating_point, input.conditions);

        const actuators::ActuatorInputsVector_T<double> u_0 = actuators::unpack_actuator_inputs_T(input.operating_point.input);
        const actuators::ActuatorLimitsVector limits = actuators::unpack_actuator_limits(input.model.actuator_limits);

        const qp::Problem problem{
            .hessian = E.transpose() * Q * E + R,
            .gradient = -E.transpose() * Q * (input.mu - mu_0),
            .lower = limits.col(0) - u_0,
            .upper = limits.col(1) - u_0
        };

        const qp::Solution solution = solver.solve(problem);

        if (solution.status != qp::Status::Solved) {
            return input.operating_point.input;
        }

        const actuators::ActuatorInputsVector_T<double> u = u_0 + solution.x;

        return actuators::pack_actuator_inputs_T(u);
    }



    std::tuple<EffectivenessMatrix, dynamics::WrenchVector_T<double>> compute_effectiveness_matrix(autodiff::AutoDiffModel& model, const operating::OperatingPoint_T<double>& operating_point, const operating::OperatingConditions& conditions) {

        const operating::StateInputVector_T<double> xu = operating::unpack_state_input_T<double>(operating_point.state, operating_point.input);
        const auto [x, u] = operating::split_state_input_vector_T(xu);

        const dynamics::StateVector_T<CppAD::AD<double>> x_ad = autodiff::double_to_cppad_double(x);

        CppAD::eigen_vector<CppAD::AD<double>> u_tracked = autodiff::start_autodiff_tracking(u);  // start of autodiff tracking

        const actuators::ActuatorInputsVector_T<CppAD::AD<double>> u_eigen = autodiff::eigen_vector_from_cppad_vector<CppAD::AD<double>, constants::input_dim>(u_tracked);

        const dynamics::State_T<CppAD::AD<double>> xt = dynamics::pack_state_T(x_ad);
        const actuators::ActuatorInputs_T<CppAD::AD<double>> ut = actuators::pack_actuator_inputs_T(u_eigen);
        const operating::OperatingPoint_T<CppAD::AD<double>> operating_point_cppad = operating::pack_state_input_T(xt, ut);
        
        const dynamics::Wrench_T<CppAD::AD<double>> W_net = autodiff::compute_net_wrench_T<CppAD::AD<double>>(operating_point_cppad, model, conditions, constants::dt);
        const dynamics::WrenchVector_T<CppAD::AD<double>> W_net_vec = dynamics::unpack_wrench_T(W_net);

        const CppAD::eigen_vector<CppAD::AD<double>> W_net_cppad = autodiff::cppad_vector_from_eigen_vector(W_net_vec);
        
        CppAD::ADFun<double> f(u_tracked, W_net_cppad);    // end of autodiff tracking

        const constants::MatrixX_T<double, constants::virtual_input_dim, constants::input_dim> jac_full = autodiff::compute_jac<constants::virtual_input_dim, constants::input_dim>(f, u);

		// evaluate W_net_vec
  		const dynamics::WrenchVector_T<double> W_net_vec_double = autodiff::evaluate_tracked_vector<constants::virtual_input_dim, constants::input_dim>(f, u);

        return { jac_full, W_net_vec_double };
    }

}
