#pragma once
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/util/public/math.hpp"
#include "simulation/util/public/trig.hpp"

namespace aerodynamics
{

	template <typename T>
	AerodynamicState_T<T> compute_aerodynamic_state_T(
		const dynamics::Twist_T<T>& twist,
		const atmospheric::Wind& windB)
	{
		AerodynamicState_T<T> out;
		const constants::Vector3_T<T> vB_rel = twist.v - windB.data.cast<T>();
		out.Vinf = vB_rel.norm();

		if (out.Vinf > T(constants::eps)) {
			out.alpha = util::atan2(vB_rel.z(), vB_rel.x());
			out.beta = util::asin(util::clamp_to_1(vB_rel.y() / out.Vinf));
		}

		return out;
	}

} // namespace aerodynamics
