#pragma once
#include "simulation/dynamics/public/data/types.hpp"

namespace frames {
	struct Frame;
} // namespace frames

namespace dynamics {

	RigidBodyState get_rigid_body_state(const frames::Frame& F);

	RigidBodyState compute_rigid_body_state(const frames::Frame& F, const frames::Frame& R);

	RigidBodyState invert_rigid_body_state(const RigidBodyState& X_BA);

	RigidBodyState compose_rigid_body_state(const RigidBodyState& X_BA, const RigidBodyState& X_AR);

	RigidBodyState rebase_cg_state(const RigidBodyState& X_GN, const Position& pB_GB, const frames::Frame& CGFrame);

} // namespace dynamics
