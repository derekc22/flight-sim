#include "simulation/aerodynamics/public/detail/state.hpp"
#include "simulation/dynamics/public/detail/state.hpp"

namespace aerodynamics {

    AerodynamicState compute_aerodynamic_state(const frames::Frame& F, const frames::Frame& R, const atmospheric::Wind& windB) {
        return compute_aerodynamic_state(dynamics::compute_rigid_body_state(F, R), windB);
    }

    AerodynamicState compute_aerodynamic_state(const dynamics::RigidBodyState& X, const atmospheric::Wind& windB) {
        const dynamics::Twist_T<double> twist{ .v = X.v.data, .w = X.w.data };

        const AerodynamicState_T<double> aero = compute_aerodynamic_state_T<double>(twist, windB);

        return { FreeStreamVelocity{ aero.Vinf }, AngleOfAttack{ aero.alpha }, SideslipAngle{ aero.beta } };
    }

}
