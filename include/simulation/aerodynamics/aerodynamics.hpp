#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include "simulation/structural/structural.hpp"
#include <simulation/dynamics/dynamics.hpp>
#include <simulation/atmospheric/atmospheric.hpp>

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


    struct ControlInputs {
        double elevator = 0.0;  // rad
        double aileron = 0.0;   // rad, signed command
        double rudder = 0.0;    // rad
        double flap = 0.0;      // rad
        double spoiler = 0.0;   // rad
    };


    struct DynamicDerivativeSet {
        // coefficient derivatives with respect to normalized body rates
        // p_hat = p b / (2 V), q_hat = q c / (2 V), r_hat = r b / (2 V)
        double CL_qhat = 0.0;
        double CD_qhat = 0.0;
        double CM_qhat = 0.0;

        double CL_phat = 0.0;
        double CL_rhat = 0.0;
        double CD_phat = 0.0;
        double CD_rhat = 0.0;
        double CM_phat = 0.0;
        double CM_rhat = 0.0;
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
        DynamicDerivativeSet dyn;

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

    struct SurfaceKinematics {
        Eigen::Vector3d r_ac_B  = global::Zero3;   // CG -> AC
        Eigen::Vector3d v_rel_B = global::Zero3;   // local air-relative velocity at surface
        double V = 0.0;                                      // local speed magnitude
        double qbar = 0.0;                                   // dynamic pressure
        double alpha = 0.0;                                  // local alpha
        double p_hat = 0.0;
        double q_hat = 0.0;
        double r_hat = 0.0;
    };

    struct SurfaceCoefficients {
        LiftCoefficient CL;
        DragCoefficient CD;
        MomentCoefficient CM;
    };

    struct AerodynamicLoad {
        dynamics::Force F;
        dynamics::Moment M;
    };

    struct AerodynamicProperties {
        std::vector<Surface> surfaces;
        std::unordered_map<std::string, size_t> surfaceIDs;

        AerodynamicProperties(std::vector<Surface> s);

        void compute_aero_properties();
        std::unordered_map<std::string, size_t> build_IDs();

    };

    SurfaceKinematics compute_surface_kinematics(
        const Surface& s,
        const structural::StructuralProperties& structuralProperties,
        const dynamics::RigidBodyState& rigidBodyState,
        const atmospheric::Density& rho,
        const atmospheric::Wind& windB
    );

    SurfaceCoefficients compute_surface_coefficients( const Surface& s, const SurfaceKinematics& sk, const ControlInputs& u);

    AerodynamicLoad compute_surface_loads(const Surface& s,const SurfaceKinematics& sk,const SurfaceCoefficients& sc);

    AerodynamicLoad step_aero_forces_moments(
        const AerodynamicProperties& aerodynamicProperties,
        const structural::StructuralProperties& structuralProperties,
        const dynamics::RigidBodyState& rigidBodyState,
        const atmospheric::Density& rho,
        const ControlInputs& u,
        const atmospheric::Wind& windB
    );
    

    AerodynamicState compute_aerodynamic_state(const dynamics::RigidBodyState& rigidBodyState, const atmospheric::Wind& windB);


    dynamics::OrientationMatrix CBS(const aerodynamics::AngleOfAttack& alpha);
    dynamics::OrientationMatrix CSW(const aerodynamics::SideslipAngle& beta);



}