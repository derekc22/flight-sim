#pragma once
#include "simulation/dynamics/public/data/types.hpp"

namespace integrators {

	dynamics::RigidBodyStateDot compute_rigid_body_state_dot(const dynamics::RigidBodyState& Xt,
	    const dynamics::Mass& mass,
	    const dynamics::InertiaTensor& JB,
	    const dynamics::Wrench& WB_net_t);

	dynamics::RigidBodyState add_scaled_rigid_body_state_dot(const dynamics::RigidBodyState& X,
	    const dynamics::RigidBodyStateDot& X_dot,
	    double scale);

	dynamics::RigidBodyState add_rk4_weighted_rigid_body_state_dot(const dynamics::RigidBodyState& X,
	    const dynamics::RigidBodyStateDot& k1,
	    const dynamics::RigidBodyStateDot& k2,
	    const dynamics::RigidBodyStateDot& k3,
	    const dynamics::RigidBodyStateDot& k4,
	    double dt);

} // namespace integrators
