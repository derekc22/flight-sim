#include <cmath>
#include <tuple>
#include <vector>
#include <spdlog/spdlog.h>
#include "simulation/allocator/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/control/public.hpp"
#include "simulation/autodiff/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/operating/public.hpp"

namespace allocator {

    control::ControlOutputSet AllocatorProperties::step(const AllocatorInput& input) {

        auto [E, mu_0] = compute_effectiveness_matrix(input.model, input.operating_point, input.conditions);

        const actuators::ActuatorInputsVector_T<double> u_0 = actuators::unpack_actuator_inputs_T(input.operating_point.input);

        dynamics::WrenchVector_T<double> mu_actuator_0 = mu_0;
        if (input.u_preferred.has_value()) {
            const operating::OperatingPoint_T<double> preferred_operating_point{
                .state = input.operating_point.state,
                .input = input.u_preferred.value()
            };
            const dynamics::Wrench_T<double> mu_preferred = autodiff::compute_net_wrench_T<double>(preferred_operating_point, input.model, input.conditions, constants::dt);
            mu_actuator_0 -= dynamics::unpack_wrench_T(mu_preferred);
        }

        dynamics::WrenchVector_T<double> err = input.mu - mu_actuator_0;
        EffectivenessMatrix E_active = E;

        // evaluate active mask
        for (Eigen::Index i = 0; i < err.rows(); ++i) {
            if (!input.active_mask[i]) {
                err(i) = 0.0;
                E_active.row(i).setZero();
            }
        }

        constants::MatrixX_T<double, constants::input_dim, constants::input_dim> trim_hessian = constants::MatrixX_T<double, constants::input_dim, constants::input_dim>::Zero();
        actuators::ActuatorInputsVector_T<double> trim_gradient = actuators::ActuatorInputsVector_T<double>::Zero();
        actuators::ActuatorInputsVector_T<double> actuator_target = u_0;
        if (input.u_preferred.has_value()) {
            const actuators::ActuatorInputsVector_T<double> u_preferred = actuators::unpack_actuator_inputs_T(input.u_preferred.value());
            trim_hessian = R;
            trim_gradient = R * (u_0 - u_preferred);
            actuator_target = u_preferred;
        }

        const constants::MatrixX_T<double, constants::input_dim, constants::input_dim> hessian = E_active.transpose() * Q * E_active + R + trim_hessian;
        const actuators::ActuatorInputsVector_T<double> gradient = -E_active.transpose() * Q * err + trim_gradient;
        const actuators::ActuatorLimitsVector limits = actuators::unpack_actuator_limits(input.model.actuator_limits);

        control::ControlOutput u_constrained = solve_qp(hessian, gradient, u_0, actuator_target, limits, input.actuator_mask, true);
        control::ControlOutput u_unconstrained = solve_qp(hessian, gradient, u_0, actuator_target, limits, input.actuator_mask, false);

        actuators::ActuatorInputsVector_T<double> u_constrained_vec = actuators::unpack_actuator_inputs_T(u_constrained);
        actuators::ActuatorInputsVector_T<double> u_unconstrained_vec = actuators::unpack_actuator_inputs_T(u_unconstrained);

        bool allocation_limited = false;
        for (Eigen::Index i = 0; i < u_unconstrained_vec.rows(); ++i) {
            if (!input.actuator_mask[i] || std::abs(limits(i, 1) - limits(i, 0)) <= constants::eps) {
                continue;
            }

            const double tolerance = constants::eps * (1.0 + std::abs(limits(i, 0)) + std::abs(limits(i, 1)));
            if (u_unconstrained_vec(i) < limits(i, 0) - tolerance || u_unconstrained_vec(i) > limits(i, 1) + tolerance) {
                allocation_limited = true;
                break;
            }
        }

        dynamics::WrenchVector_T<double> delta_mu_vec = dynamics::WrenchVector_T<double>::Zero();
        if (allocation_limited) {
            delta_mu_vec = E_active * (u_constrained_vec - u_unconstrained_vec);
        }

        return { u_constrained, delta_mu_vec };
    }


    control::ControlOutput AllocatorProperties::solve_qp(const constants::MatrixX_T<double, constants::input_dim, constants::input_dim>& hessian, const actuators::ActuatorInputsVector_T<double>& gradient, const actuators::ActuatorInputsVector_T<double>& u_0, const actuators::ActuatorInputsVector_T<double>& actuator_target, const actuators::ActuatorLimitsVector& limits, const std::array<bool, constants::input_dim>& actuator_mask, bool constrained) {
        qp::Solution solution;

        if (constrained) {
            Eigen::VectorXd lower = limits.col(0) - u_0;
            Eigen::VectorXd upper = limits.col(1) - u_0;

            for (Eigen::Index i = 0; i < lower.rows(); ++i) {
                if (!actuator_mask[i]) {
                    lower(i) = actuator_target(i) - u_0(i);
                    upper(i) = lower(i);
                }
            }

            const qp::Problem problem{
                .hessian = hessian,
                .gradient = gradient,
                .lower = lower,
                .upper = upper
            };

            solution = solver.solve(problem);
        }
        else {
            std::vector<Eigen::Index> free_indices;
            actuators::ActuatorInputsVector_T<double> x = actuators::ActuatorInputsVector_T<double>::Zero();

            for (Eigen::Index i = 0; i < x.rows(); ++i) {
                if (std::abs(limits(i, 1) - limits(i, 0)) <= constants::eps) {
                    x(i) = limits(i, 0) - u_0(i);
                }
                else if (!actuator_mask[i]) {
                    x(i) = actuator_target(i) - u_0(i);
                }
                else {
                    free_indices.push_back(i);
                }
            }

            if (!free_indices.empty()) {
                Eigen::MatrixXd hessian_free(free_indices.size(), free_indices.size());
                Eigen::VectorXd gradient_free(free_indices.size());

                for (std::size_t i = 0; i < free_indices.size(); ++i) {
                    gradient_free(i) = gradient(free_indices[i]) + hessian.row(free_indices[i]).dot(x);

                    for (std::size_t j = 0; j < free_indices.size(); ++j) {
                        hessian_free(i, j) = hessian(free_indices[i], free_indices[j]);
                    }
                }

                const Eigen::VectorXd x_free = hessian_free.completeOrthogonalDecomposition().solve(-gradient_free);

                for (std::size_t i = 0; i < free_indices.size(); ++i) {
                    x(free_indices[i]) = x_free(i);
                }
            }

            solution = {
                .x = x,
                .status = qp::Status::Solved
            };
        }

        if (solution.status != qp::Status::Solved) {
            spdlog::error("allocator::AllocatorProperties::step QP solve failed with status {}", static_cast<int>(solution.status));
            return actuators::pack_actuator_inputs_T(u_0);
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


    AllocatorInput build_allocator_input(
        const control::VirtualControlOutput& mu_cmd,
        const std::array<bool, constants::virtual_input_dim>& active_mask,
        const std::array<bool, constants::input_dim>& actuator_mask,
        const dynamics::RigidBodyState& Zt, 
        const control::ControlOutput& u_actual_t_1,
        const std::optional<control::ControlOutput>& u_preferred,
        const operating::OperatingConditions& conditions, 
        autodiff::AutoDiffModel& model
    ) {
        return {
            .mu = dynamics::unpack_wrench(mu_cmd),
            .active_mask = active_mask,
            .actuator_mask = actuator_mask,
            .operating_point = {
                .state = dynamics::pack_state(Zt),
                .input = u_actual_t_1
            },
            .u_preferred = u_preferred,
            .conditions = conditions,
            .model = model
        };
    }

}
