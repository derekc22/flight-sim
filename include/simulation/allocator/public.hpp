#pragma once
#include <tuple>
#include "simulation/control/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/qp/public.hpp"

namespace allocator {

	using EffectivenessMatrix = constants::MatrixX_T<double, constants::virtual_input_dim, constants::input_dim>;

	struct AllocatorInput {
		control::VirtualControlOutputVector_T<double> mu;
		operating::OperatingPoint_T<double> operating_point;	// (xt, ut_1)
        operating::OperatingConditions conditions;
        autodiff::AutoDiffModel& model;
    };

    struct AllocatorProperties {
        constants::MatrixX_T<double, constants::virtual_input_dim, constants::virtual_input_dim> Q = constants::IX_T<double, constants::virtual_input_dim>;
        constants::MatrixX_T<double, constants::input_dim, constants::input_dim> R = 1e-6 * constants::IX_T<double, constants::input_dim>;
        qp::Solver solver{constants::input_dim};

        control::ControlOutput step(const AllocatorInput& input);
    };

	std::tuple<EffectivenessMatrix, dynamics::WrenchVector_T<double>> compute_effectiveness_matrix(autodiff::AutoDiffModel& model, const operating::OperatingPoint_T<double>& operating_point, const operating::OperatingConditions& conditions);

}
