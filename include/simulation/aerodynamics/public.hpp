#pragma once
#include <cstddef>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include "simulation/actuators/surface/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/frames/public.hpp"
#include "simulation/util/public.hpp"

namespace aerodynamics {
    struct DynamicDerivatives {
        double CL_qhat = 0.0;
        double CD_qhat = 0.0;
        double CM_qhat = 0.0;
        double CL_phat = 0.0;
        double CD_phat = 0.0;
        double CM_phat = 0.0;
        double CL_rhat = 0.0;
        double CD_rhat = 0.0;
        double CM_rhat = 0.0;
    };

    struct ControlDerivatives {
        double dCL_de = 0.0;
        double dCM_de = 0.0;
        double dCD_de = 0.0;
        double dCL_da = 0.0;
        double dCM_da = 0.0;
        double dCD_da = 0.0;
        double dCL_dr = 0.0;
        double dCM_dr = 0.0;
        double dCD_dr = 0.0;
        double dCL_df = 0.0;
        double dCM_df = 0.0;
        double dCD_df = 0.0;
        double dCL_ds = 0.0;
        double dCM_ds = 0.0;
        double dCD_ds = 0.0;
    };

    struct Surface {
        std::string id;
        double chord;
        double span;
        double area;
        double AR;
        // For a thin airfoil in incompressible subsonic flow, the aerodynamic center is approximately at the quarter chord
        Eigen::Vector3d p_ac;
        Eigen::Vector3d n;
        double CL0, e, i, CD0, CDa, a0, CM0, CMa;
        DynamicDerivatives dyn;
        ControlDerivatives ctrl;
    };

    template <typename T>
    struct SurfaceKinematics_T {
        constants::Vector3_T<T> vB_rel = constants::Zero3_T<T>;
        T V = T(0.0);
        T qbar = T(0.0);
        T alpha = T(0.0);
        T p_hat = T(0.0);
        T q_hat = T(0.0);
        T r_hat = T(0.0);
    };

    template <typename T>
    struct SurfaceCoefficients_T {
        T CL = T(0.0);
        T CD = T(0.0);
        T CM = T(0.0);
    };

    struct AerodynamicProperties {
        std::vector<Surface> surfaces;

        AerodynamicProperties(std::vector<Surface> s);
        void compute_surface_geometry();
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

    template <typename T>
    struct AerodynamicState_T {
        T Vinf = T(0.0);
        T alpha = T(0.0);
        T beta = T(0.0);
    };

    template <typename T>
    SurfaceKinematics_T<T> compute_surface_kinematics_T(const Surface& s, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& atm, const atmospheric::Wind& windB);

    template <typename T>
    SurfaceCoefficients_T<T> compute_surface_coefficients_T(const Surface& s, const SurfaceKinematics_T<T>& sk, const actuators::SurfaceActuatorInputs_T<T>& u);

    template <typename T>
    dynamics::Wrench_T<T> compute_surface_loads_T(const Surface& s, const SurfaceKinematics_T<T>& sk, const SurfaceCoefficients_T<T>& sc);

    AerodynamicState compute_aerodynamic_state(const frames::Frame& F, const frames::Frame& R, const atmospheric::Wind& windB);

    template <typename T>
    dynamics::Wrench_T<T> step_aero_forces_moments_T(const AerodynamicProperties& aerodynamic_properties, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& atm, const actuators::SurfaceActuatorInputs_T<T>& u, const atmospheric::Wind& windB);

    template <typename T>
    AerodynamicState_T<T> compute_aerodynamic_state_T(const dynamics::Twist_T<T>& twist, const atmospheric::Wind& windB);

    dynamics::Wrench step_aero_forces_moments(const AerodynamicProperties& aerodynamic_properties, const dynamics::RigidBodyState& X, const atmospheric::StaticAtmosphericState& atm, const actuators::SurfaceActuatorInputs_T<double>& u, const atmospheric::Wind& windB);

    AerodynamicState compute_aerodynamic_state(const dynamics::RigidBodyState& X, const atmospheric::Wind& windB);

    dynamics::OrientationMatrix CBS(const AngleOfAttack& alpha);
    dynamics::OrientationMatrix CSW(const SideslipAngle& beta);

}
#include "simulation/aerodynamics/public.tpp"
