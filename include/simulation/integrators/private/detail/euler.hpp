#pragma once
#include "simulation/dynamics/public/data/types.hpp"

namespace integrators
{

	dynamics::EulerAngles
	eul_kin(const dynamics::EulerAngles& eul_t, const dynamics::EulerAngleRates& eul_dot_t, double dt);

	dynamics::OrientationMatrix
	rot_kin(const dynamics::OrientationMatrix& CIB_t, const dynamics::AngularVelocity& wB_BI_t, double dt);

	dynamics::TranslationalVelocity trans_dyn_vel(const dynamics::TranslationalVelocity& vB_BI_t,
		const dynamics::AngularVelocity& wB_BI_t,
		const dynamics::Mass& mass,
		const dynamics::Force& FB_net_t,
		double dt);

	dynamics::AngularVelocity rot_dyn(const dynamics::AngularVelocity& wB_BI_t,
		const dynamics::InertiaTensor& JB,
		const dynamics::Moment& MB_net_t,
		double dt);

} // namespace integrators
