#pragma once
#include "simulation/aerodynamics/public/components/effector.hpp"
#include "simulation/aerodynamics/public/data/types.hpp"

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace aerodynamics
{

	struct Surface {
		std::string id;
		double chord;
		double span;
		double area;
		double AR;
		// For a thin airfoil in incompressible subsonic flow, the aerodynamic center is approximately at the quarter chord
		Eigen::Vector3d pB_acB;
		Eigen::Vector3d n;
		double CL0, e, i, CD0, CDa, a0, CM0, CMa;
		DynamicDerivatives dyn;
		std::vector<SurfaceEffector> effectors;

		void compute_geometry();

		template <typename T> SurfaceKinematics_T<T> compute_kinematics(const SurfaceInput_T<T>& input);

		template <typename T>
		SurfaceCoefficients_T<T> compute_coefficients(const SurfaceKinematics_T<T>& sk,
			const actuators::SurfaceActuatorInputs_T<T>& u);

		template <typename T>
		dynamics::Wrench_T<T> compute_loads(const constants::Vector3_T<T>& pB_GB,
			const SurfaceKinematics_T<T>& sk,
			const SurfaceCoefficients_T<T>& sc);

		template <typename T> dynamics::Wrench_T<T> step(const SurfaceInput_T<T>& input);
	};

} // namespace aerodynamics

#include "simulation/aerodynamics/public/components/surface.tpp"
