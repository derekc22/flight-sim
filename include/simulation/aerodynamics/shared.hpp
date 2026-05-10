#pragma once
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/atmospheric/shared.hpp"
#include "simulation/dynamics/shared.hpp"

namespace frames { struct Frame; } // forward declare
namespace structural { struct StructuralProperties; } // forward declare
namespace aerodynamics { struct AerodynamicProperties; } // forward declare

namespace aerodynamics {

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

    struct AerodynamicWrench : dynamics::Wrench {};

    template <typename T>
    struct AerodynamicState_T {
        T Vinf = T(0);
        T alpha = T(0);
        T beta = T(0);
    };

    /** @warning The parent of F must be an inertial frame: ECEFFrame or NEDFrameECEF */
    AerodynamicState aerodynamic_state(const frames::Frame& F, const atmospheric::Wind& windB);

    template <typename T>
    dynamics::Wrench_T<T> step_aero_forces_moments_T(const AerodynamicProperties& aerodynamic_properties, const structural::StructuralProperties& structural_properties, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& static_atm_state, const actuators::SurfaceActuatorInputs_T<T>& u, const atmospheric::Wind& windB);

    template <typename T>
    AerodynamicState_T<T> compute_aerodynamic_state_T(const dynamics::Twist_T<T>& twist, const atmospheric::Wind& windB);

    AerodynamicWrench step_aero_forces_moments(const AerodynamicProperties& aerodynamic_properties, const structural::StructuralProperties& structural_properties, const dynamics::RigidBodyState& rigid_body_state, const atmospheric::StaticAtmosphericState& static_atm_state, const actuators::SurfaceActuatorInputs_T<double>& u, const atmospheric::Wind& windB);
    AerodynamicState compute_aerodynamic_state(const dynamics::RigidBodyState& rigid_body_state, const atmospheric::Wind& windB);

    dynamics::OrientationMatrix CBS(const aerodynamics::AngleOfAttack& alpha);
    dynamics::OrientationMatrix CSW(const aerodynamics::SideslipAngle& beta);

}
