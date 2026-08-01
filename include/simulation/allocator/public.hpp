#pragma once
#include <tuple>
#include <array>
#include "simulation/control/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/qp/public.hpp"

namespace allocator {

	using EffectivenessMatrix = constants::MatrixX_T<double, constants::virtual_input_dim, constants::input_dim>;

	struct AllocatorInput {
		control::VirtualControlOutputVector_T<double> mu;
		std::array<bool, constants::virtual_input_dim> active_mask;
		operating::OperatingPoint_T<double> operating_point;	// (zt, ut_1)
        operating::OperatingConditions conditions;
        autodiff::AutoDiffModel& model;
    };

    struct AllocatorProperties {
        constants::MatrixX_T<double, constants::virtual_input_dim, constants::virtual_input_dim> Q;
        constants::MatrixX_T<double, constants::input_dim, constants::input_dim> R;
        qp::Solver solver{constants::input_dim};

        control::ControlOutputSet step(const AllocatorInput& input);
        std::tuple<control::ControlOutput, EffectivenessMatrix> solve_qp(const AllocatorInput& input, bool constrained);
    };

	std::tuple<EffectivenessMatrix, dynamics::WrenchVector_T<double>> compute_effectiveness_matrix(autodiff::AutoDiffModel& model, const operating::OperatingPoint_T<double>& operating_point, const operating::OperatingConditions& conditions);

    AllocatorInput build_allocator_input(
        const control::VirtualControlOutput& mu_cmd,
        const std::array<bool, constants::virtual_input_dim>& active_mask,
        const dynamics::RigidBodyState& Zt,
        const control::ControlOutput& u_actual_t_1,
        const operating::OperatingConditions& conditions, 
        autodiff::AutoDiffModel& model
    );

    EffectivenessMatrix apply_beta_scale(const EffectivenessMatrix& E, const actuators::ActuatorInputsVector_T<double>& time_constants);

}
