#pragma once
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/dynamics/public/data/types.hpp"

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace aerodynamics
{

	struct DynamicDerivative {
		double dCL = 0.0;
		double dCD = 0.0;
		double dCM = 0.0;
	};

	struct DynamicDerivatives {
		DynamicDerivative p_hat;
		DynamicDerivative q_hat;
		DynamicDerivative r_hat;
	};

	struct SurfaceEffector {
		actuators::SurfaceActuatorID actuator_id;
		double dCL;
		double dCD;
		double dCM;
	};

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
	};

	template <typename T> struct SurfaceKinematics_T {
		constants::Vector3_T<T> vB_rel = constants::Zero3_T<T>;
		T V = T(0.0);
		T qbar = T(0.0);
		T alpha = T(0.0);
		T p_hat = T(0.0);
		T q_hat = T(0.0);
		T r_hat = T(0.0);
	};

	template <typename T> struct SurfaceCoefficients_T {
		T CL = T(0.0);
		T CD = T(0.0);
		T CM = T(0.0);
	};

	struct FreeStreamVelocity {
		double data;
	};

	struct AngleOfAttack {
		double data;
	};

	struct SideslipAngle {
		double data;
	};

	struct AerodynamicState {
		FreeStreamVelocity Vinf;
		AngleOfAttack alpha;
		SideslipAngle beta;
	};

	template <typename T> struct AerodynamicState_T {
		T Vinf = T(0.0);
		T alpha = T(0.0);
		T beta = T(0.0);
	};

	template <typename T> struct AerodynamicsManagerInput_T {
		const constants::Vector3_T<T>& pB_GB;
		const dynamics::Twist_T<T>& twist;
		const atmospheric::StaticAtmosphericState& atm;
		const actuators::SurfaceActuatorInputs_T<T>& u;
		const atmospheric::Wind& windB;
	};

	template <typename T> struct AerodynamicsManagerOutput_T {
		dynamics::Wrench_T<T> WB_aerodynamic;
	};

} // namespace aerodynamics
