#pragma once
#include <vector>
#include "simulation/constants/public/linalg.hpp"
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/aerodynamics/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"

namespace aerodynamics {

	template <typename T>
	SurfaceKinematics_T<T> compute_surface_kinematics_T(const Surface& s,
	    const constants::Vector3_T<T>& pB_GB,
	    const dynamics::Twist_T<T>& twist,
	    const atmospheric::StaticAtmosphericState& atm,
	    const atmospheric::Wind& windB);

	template <typename T>
	SurfaceCoefficients_T<T> compute_surface_coefficients_T(const Surface& s,
	    const SurfaceKinematics_T<T>& sk,
	    const actuators::SurfaceActuatorInputs_T<T>& u);

	template <typename T>
	dynamics::Wrench_T<T> compute_surface_loads_T(const Surface& s,
	    const constants::Vector3_T<T>& pB_GB,
	    const SurfaceKinematics_T<T>& sk,
	    const SurfaceCoefficients_T<T>& sc);

	template <typename T>
	dynamics::Wrench_T<T> step_aero_forces_moments_T(const std::vector<Surface>& surfaces,
	    const constants::Vector3_T<T>& pB_GB,
	    const dynamics::Twist_T<T>& twist,
	    const atmospheric::StaticAtmosphericState& atm,
	    const actuators::SurfaceActuatorInputs_T<T>& u,
	    const atmospheric::Wind& windB);

	dynamics::Wrench step_aero_forces_moments(const std::vector<Surface>& surfaces,
	    const structural::CenterOfGravity& pB_GB,
	    const dynamics::RigidBodyState& X,
	    const atmospheric::StaticAtmosphericState& atm,
	    const actuators::SurfaceActuatorInputs_T<double>& u,
	    const atmospheric::Wind& windB);

} // namespace aerodynamics

#include "simulation/aerodynamics/public/detail/loads.tpp"
