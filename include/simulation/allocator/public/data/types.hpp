#pragma once
#include <array>
#include <optional>
#include "simulation/control/public/data/types.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/operating/public/data/types.hpp"

namespace autodiff { struct AutoDiffModel; }

namespace allocator {

	struct AllocatorManagerInput {
		control::VirtualControlOutputVector_T<double> mu;
		std::array<bool, constants::virtual_input_dim> active_mask;
		std::array<bool, constants::input_dim> actuator_mask;
		operating::OperatingPoint_T<double> operating_point;	// (zt, ut_1)
        std::optional<control::ControlOutput> u_preferred;
        operating::OperatingConditions conditions;
        autodiff::AutoDiffModel& model;
    };

    struct AllocatorManagerOutput {
        control::ControlOutput u;
        dynamics::WrenchVector_T<double> delta_mu_vec_t_1;
    };

}
