#include <cmath>
#include <vector>
#include <spdlog/spdlog.h>
#include "simulation/allocator/private/detail/solve.hpp"

namespace allocator {

    control::ControlOutput solve_qp_constrained(qp::Solver& solver, const constants::MatrixX_T<double, constants::input_dim, constants::input_dim>& hessian, const actuators::ActuatorInputsVector_T<double>& gradient, const actuators::ActuatorInputsVector_T<double>& u_0, const actuators::ActuatorInputsVector_T<double>& actuator_target, const actuators::ActuatorLimitsVector& limits, const std::array<bool, constants::input_dim>& actuator_mask) {
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

        const qp::Solution solution = solver.solve(problem);

        if (solution.status != qp::Status::Solved) {
            spdlog::error("allocator::AllocatorManager::step QP solve failed with status {}", static_cast<int>(solution.status));
            return actuators::pack_actuator_inputs_T(u_0);
        }

        const actuators::ActuatorInputsVector_T<double> u = u_0 + solution.x;

        return actuators::pack_actuator_inputs_T(u);
    }

    control::ControlOutput solve_qp_unconstrained(const constants::MatrixX_T<double, constants::input_dim, constants::input_dim>& hessian, const actuators::ActuatorInputsVector_T<double>& gradient, const actuators::ActuatorInputsVector_T<double>& u_0, const actuators::ActuatorInputsVector_T<double>& actuator_target, const actuators::ActuatorLimitsVector& limits, const std::array<bool, constants::input_dim>& actuator_mask) {
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

        const actuators::ActuatorInputsVector_T<double> u = u_0 + x;

        return actuators::pack_actuator_inputs_T(u);
    }

}
