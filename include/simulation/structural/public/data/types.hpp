#pragma once
#include "simulation/constants/public/linalg.hpp"
#include "simulation/dynamics/public/data/types.hpp"

#include <Eigen/Dense>

namespace structural
{

	struct CenterOfGravity {
		Eigen::Vector3d data; // pB_GB [m]
	};

	struct StructuralState {
		dynamics::Mass mass;
		CenterOfGravity pB_GB;
		dynamics::InertiaTensor JB;
	};

	struct StructuralManagerInput {};

	struct StructuralManagerOutput {
		StructuralState struc_t;
	};

	template <typename T> struct StructuralState_T {
		T mass = T(0.0);
		constants::Vector3_T<T> pB_GB = constants::Zero3_T<T>;
		constants::Matrix3_T<T> JB = constants::Zero3x3_T<T>;
	};

} // namespace structural
