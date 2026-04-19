#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include "simulation/structural/structural.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"
#include <simulation/dynamics/dynamics.hpp>
#include <simulation/atmospheric/atmospheric.hpp>

namespace frames { struct Frame; } // forward declare
namespace control { struct SurfaceActuatorInputs; } // forward declare

namespace aerodynamics {

    struct FreeStreamVelocity {
        double data; // freestream velocity, V∞​ [ms^-1]
    };

    struct LiftCoefficient {
        double data;    // [dimensionless]
    };

    struct DragCoefficient {
        double data;    // [dimensionless]
    };

    struct MomentCoefficient {
        double data;    // [dimensionless]
    };

    struct AngleOfAttack {
        double data;    // [rad]
    };

    struct SideslipAngle {
        double data;    // [rad]
    };

    struct AerodynamicState {
        FreeStreamVelocity Vinf;    
        AngleOfAttack alpha;   
        SideslipAngle beta;
    };

    /** @warning The parent of F must be an inertial frame: ECEFFrame or NEDFrameECEF */
    AerodynamicState aerodynamic_state(const frames::Frame& F, const atmospheric::Wind& windB);

    struct DynamicDerivatives {
        // coefficient derivatives with respect to normalized body rates
        // p_hat = p b / (2 V), q_hat = q c / (2 V), r_hat = r b / (2 V)
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
        // Define u > 0 -> pitch up, u < 0 -> pitch down
        double dCL_de = 0.0;    // dCL_de < 0
        double dCM_de = 0.0;    // dCM_de > 0
        double dCD_de = 0.0;    // dCD_de > 0

        // Define u > 0 -> roll right, u < 0 -> roll left
        double dCL_da = 0.0;    // dCL_da_right < 0, dCL_da_left > 0
        double dCM_da = 0.0;    // dCM_da_right ≈ 0, dCM_da_left ≈ 0
        double dCD_da = 0.0;    // dCD_da_right > 0, dCD_da_left > 0

        // Define u > 0 -> yaw right, u < 0 -> yaw left
        double dCL_dr = 0.0;    // dCL_dr < 0
        double dCM_dr = 0.0;    // dCM_dr > 0
        double dCD_dr = 0.0;    // dCD_dr > 0

        // Define u > 0 -> flap deployed
        double dCL_df = 0.0;    // dCL_df > 0
        double dCM_df = 0.0;    // dCM_df < 0
        double dCD_df = 0.0;    // dCD_df > 0

        // Define u > 0 -> spoiler deployed
        double dCL_ds = 0.0;    // dCL_ds < 0
        double dCM_ds = 0.0;    // dCM_ds ≈ 0
        double dCD_ds = 0.0;    // dCD_ds > 0
    };

    struct Surface {
        std::string id;

        double chord;
        double span;
        double area;
        double AR;

        Eigen::Vector3d p_ref;   // reference location from geometry
        Eigen::Vector3d p_ac;    // quarter-chord aerodynamic center
        Eigen::Vector3d n;       // unit-ish surface normal in body axes

        // static coefficient model
        double CL0, e, i, CD0, CDa, a0, CM0, CMa;

        // optional higher-order effects
        DynamicDerivatives dyn;
        ControlDerivatives ctrl;
    };

    struct SurfaceKinematics {
        Eigen::Vector3d r_ac_B  = constants::Zero3;   // CG -> AC
        Eigen::Vector3d v_rel_B = constants::Zero3;   // local air-relative velocity at surface
        double V = 0.0;                               // local speed magnitude
        double qbar = 0.0;                            // dynamic pressure
        double alpha = 0.0;                           // local alpha
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
    struct SurfaceActuatorInputs_T {
        T elevator_cmd = T(0);
        T aileron_cmd = T(0);
        T rudder_cmd = T(0);
        T flap_cmd = T(0);
        T spoiler_cmd = T(0);
    };

    template <typename T>
    struct SurfaceKinematics_T {
        constants::Vector3_T<T> r_ac_B =  constants::Zero3_T<T>;
        constants::Vector3_T<T> v_rel_B = constants::Zero3_T<T>;
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

    template <typename T>
    struct AerodynamicWrench_T {
        constants::Vector3_T<T> F = constants::Zero3_T<T>;
        constants::Vector3_T<T> M = constants::Zero3_T<T>;
    };

    template <typename T>
    struct AerodynamicState_T {
        T Vinf = T(0);
        T alpha = T(0);
        T beta = T(0);
    };

    template <typename T>
    SurfaceKinematics_T<T> compute_surface_kinematics_T(const Surface& s, const structural::StructuralProperties& structural_properties, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& static_atmospheric_state, const atmospheric::Wind& windB);

    template <typename T>
    SurfaceCoefficients_T<T> compute_surface_coefficients_T(const Surface& s, const SurfaceKinematics_T<T>& sk, const SurfaceActuatorInputs_T<T>& u);

    template <typename T>
    AerodynamicWrench_T<T> compute_surface_loads_T(const Surface& s, const SurfaceKinematics_T<T>& sk, const SurfaceCoefficients_T<T>& sc);

    template <typename T>
    AerodynamicWrench_T<T> step_aero_forces_moments_T(const AerodynamicProperties& aerodynamic_properties, const structural::StructuralProperties& structural_properties, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& static_atmospheric_state, const SurfaceActuatorInputs_T<T>& u, const atmospheric::Wind& windB);

    template <typename T>
    AerodynamicState_T<T> compute_aerodynamic_state_T(const dynamics::Twist_T<T>& twist, const atmospheric::Wind& windB);

    /** @deprecated */
    // SurfaceKinematics compute_surface_kinematics(
    //     const Surface& s,
    //     const structural::StructuralProperties& structural_properties,
    //     const dynamics::RigidBodyState& rigid_body_state,
    //     const atmospheric::StaticAtmosphericState& static_atmospheric_state,
    //     const atmospheric::Wind& windB
    // );

    /** @deprecated */
    // SurfaceCoefficients compute_surface_coefficients(
    //     const Surface& s,
    //     const SurfaceKinematics& sk,
    //     const control::SurfaceActuatorInputs& u
    // );

    /** @deprecated */
    // AerodynamicWrench compute_surface_loads(const Surface& s,const SurfaceKinematics& sk,const SurfaceCoefficients& sc);

    AerodynamicWrench step_aero_forces_moments(
        const AerodynamicProperties& aerodynamic_properties,
        const structural::StructuralProperties& structural_properties,
        const dynamics::RigidBodyState& rigid_body_state,
        const atmospheric::StaticAtmosphericState& static_atmospheric_state,
        const control::SurfaceActuatorInputs& u,
        const atmospheric::Wind& windB
    );
    

    AerodynamicState compute_aerodynamic_state(const dynamics::RigidBodyState& rigid_body_state, const atmospheric::Wind& windB);


    dynamics::OrientationMatrix CBS(const aerodynamics::AngleOfAttack& alpha);
    dynamics::OrientationMatrix CSW(const aerodynamics::SideslipAngle& beta);
}

#include "simulation/aerodynamics/aerodynamics.tpp"
