#pragma once
#include <Eigen/Dense>
#include <string>
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/structural/public/data/types.hpp"

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
        Eigen::Vector3d pB_acB;
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

    struct AerodynamicsManagerInput {
        const structural::CenterOfGravity& pB_GB;
        const dynamics::RigidBodyState& X;
        const atmospheric::StaticAtmosphericState& atm;
        const actuators::SurfaceActuatorInputs_T<double>& u;
        const atmospheric::Wind& windB;
    };

    struct AerodynamicsManagerOutput {
        AerodynamicState aero_t;
        dynamics::Wrench WB_aerodynamic;
    };

}
