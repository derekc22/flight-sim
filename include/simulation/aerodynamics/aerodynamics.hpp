#pragma once
#include <cstddef>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>
#include "simulation/aerodynamics/shared.hpp"
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/util/util.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/atmospheric/atmospheric.hpp"

namespace frames { struct Frame; } // forward declare

namespace aerodynamics {

    struct LiftCoefficient {
        double data;
    };

    struct DragCoefficient {
        double data;
    };

    struct MomentCoefficient {
        double data;
    };

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
        Eigen::Vector3d p_ref;
        Eigen::Vector3d p_ac;
        Eigen::Vector3d n;
        double CL0, e, i, CD0, CDa, a0, CM0, CMa;
        DynamicDerivatives dyn;
        ControlDerivatives ctrl;
    };

    struct SurfaceKinematics {
        Eigen::Vector3d rB_ac = constants::Zero3;
        Eigen::Vector3d vB_rel = constants::Zero3;
        double V = 0.0;
        double qbar = 0.0;
        double alpha = 0.0;
        double p_hat = 0.0;
        double q_hat = 0.0;
        double r_hat = 0.0;
    };

    struct SurfaceCoefficients {
        LiftCoefficient CL;
        DragCoefficient CD;
        MomentCoefficient CM;
    };

    struct AerodynamicWrench : dynamics::Wrench {};

    struct AerodynamicProperties {
        std::vector<Surface> surfaces;
        std::unordered_map<std::string, size_t> surfaceIDs;

        AerodynamicProperties(std::vector<Surface> s);
        void compute_aero_properties();
        std::unordered_map<std::string, size_t> build_IDs();
    };

    template <typename T>
    struct SurfaceKinematics_T {
        constants::Vector3_T<T> rB_ac = constants::Zero3_T<T>;
        constants::Vector3_T<T> vB_rel = constants::Zero3_T<T>;
        T V = T(0);
        T qbar = T(0);
        T alpha = T(0);
        T p_hat = T(0);
        T q_hat = T(0);
        T r_hat = T(0);
    };

    template <typename T>
    struct SurfaceCoefficients_T {
        T CL = T(0);
        T CD = T(0);
        T CM = T(0);
    };

    /** @warning The parent of F must be an inertial frame: ECEFFrame or NEDFrameECEF */
    AerodynamicState aerodynamic_state(const frames::Frame& F, const atmospheric::Wind& windB);

    template <typename T>
    SurfaceKinematics_T<T> compute_surface_kinematics_T(const Surface& s, const structural::StructuralProperties& structural_properties, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& static_atmospheric_state, const atmospheric::Wind& windB);

    template <typename T>
    SurfaceCoefficients_T<T> compute_surface_coefficients_T(const Surface& s, const SurfaceKinematics_T<T>& sk, const actuators::SurfaceActuatorInputs_T<T>& u);

    template <typename T>
    dynamics::Wrench_T<T> compute_surface_loads_T(const Surface& s, const SurfaceKinematics_T<T>& sk, const SurfaceCoefficients_T<T>& sc);

    template <typename T>
    dynamics::Wrench_T<T> step_aero_forces_moments_T(const AerodynamicProperties& aerodynamic_properties, const structural::StructuralProperties& structural_properties, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& static_atmospheric_state, const actuators::SurfaceActuatorInputs_T<T>& u, const atmospheric::Wind& windB);

    template <typename T>
    AerodynamicState_T<T> compute_aerodynamic_state_T(const dynamics::Twist_T<T>& twist, const atmospheric::Wind& windB);

    AerodynamicWrench step_aero_forces_moments(
        const AerodynamicProperties& aerodynamic_properties,
        const structural::StructuralProperties& structural_properties,
        const dynamics::RigidBodyState& rigid_body_state,
        const atmospheric::StaticAtmosphericState& static_atmospheric_state,
        const actuators::SurfaceActuatorInputs_T<double>& u,
        const atmospheric::Wind& windB
    );
    

    AerodynamicState compute_aerodynamic_state(const dynamics::RigidBodyState& rigid_body_state, const atmospheric::Wind& windB);


    dynamics::OrientationMatrix CBS(const aerodynamics::AngleOfAttack& alpha);
    dynamics::OrientationMatrix CSW(const aerodynamics::SideslipAngle& beta);
}

#include "simulation/aerodynamics/aerodynamics.tpp"
