#pragma once
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/constants/public/linalg.hpp"

#include <Eigen/Dense>

namespace linearization
{

	using StateJacobian = constants::MatrixX_T<double, constants::nx, constants::nx>;
	using InputJacobian = constants::MatrixX_T<double, constants::nx, constants::nu>;
	using FullStateInputJacobian = constants::MatrixX_T<double, constants::nx, constants::nxu>;

	using OutputJacobian = constants::MatrixX_T<double, constants::nx, constants::nx>;
	using DirectFeedthroughJacobian = constants::MatrixX_T<double, constants::nx, constants::nu>;

	struct LocalLinearization {
		StateJacobian A = StateJacobian::Zero();
		InputJacobian B = InputJacobian::Zero();
		OutputJacobian C = OutputJacobian::Identity();
		DirectFeedthroughJacobian D = DirectFeedthroughJacobian::Zero();
	};

	struct DiscretizedLocalLinearization {
		StateJacobian A = StateJacobian::Zero();
		InputJacobian B = InputJacobian::Zero();
		OutputJacobian C = OutputJacobian::Identity();
		DirectFeedthroughJacobian D = DirectFeedthroughJacobian::Zero();
	};

	using VirtualInputJacobian = constants::MatrixX_T<double, constants::nx, constants::nv>;
	using VirtualDirectFeedthroughJacobian = constants::MatrixX_T<double, constants::nx, constants::nv>;
	using FullVirtualStateInputJacobian = constants::MatrixX_T<double, constants::nx, constants::nxv>;

	struct VirtualLocalLinearization {
		StateJacobian A_virtual = StateJacobian::Zero();
		VirtualInputJacobian B_virtual = VirtualInputJacobian::Zero();
		OutputJacobian C_virtual = OutputJacobian::Identity();
		VirtualDirectFeedthroughJacobian D_virtual = VirtualDirectFeedthroughJacobian::Zero();
	};
} // namespace linearization
