#pragma once
#include "simulation/dynamics/public/data/types.hpp"

namespace integrators
{

	dynamics::Position trans_kin(const dynamics::Position& pt,
		const dynamics::TranslationalVelocity& vt,
		const dynamics::TranslationalAcceleration& at,
		double dt);

	dynamics::OrientationQuaternion
	quat_kin(const dynamics::OrientationQuaternion& qIB_t, const dynamics::AngularVelocity& wB_BI_t, double dt);

	dynamics::TranslationalVelocity trans_kin_vel(const dynamics::TranslationalVelocity& xt_dot,
		const dynamics::TranslationalAcceleration& xt_ddot,
		double dt);

	dynamics::RigidBodyState step_rigid_body(const dynamics::RigidBodyState& Xt,
		const dynamics::Mass& mass,
		const dynamics::InertiaTensor& JB,
		const dynamics::Wrench& WB_net_t,
		double dt);

} // namespace integrators
