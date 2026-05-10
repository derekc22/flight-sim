#include <cmath>
#include <format>
#include <stdexcept>
#include <Eigen/Dense>
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/aerodynamics/shared.hpp"
#include "simulation/atmospheric/shared.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/frames/frames.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/util/util.hpp"

namespace aerodynamics {

    AerodynamicState aerodynamic_state(const frames::Frame& F, const atmospheric::Wind& windB) {
        if (F.parent != nullptr && F.parent->name != "NEDFrameECEF") {
            throw std::invalid_argument(std::format("aerodynamics::aerodynamic_state: Invalid frame input, the parent of {} must be an inertial frame: ECEFFrame or NEDFrameECEF", F.name));
        }
        return aerodynamics::compute_aerodynamic_state(dynamics::compute_rigid_body_state(F), windB);
    }

    AerodynamicWrench step_aero_forces_moments(const AerodynamicProperties& aerodynamic_properties, const structural::StructuralProperties& structural_properties, const dynamics::RigidBodyState& rigid_body_state, const atmospheric::StaticAtmosphericState& static_atm_state, const actuators::SurfaceActuatorInputs_T<double>& u, const atmospheric::Wind& windB) {
        const dynamics::Twist_T<double> twist{
            .v = rigid_body_state.v.data,
            .w = rigid_body_state.w.data,
        };

        const dynamics::Wrench_T<double> loads = step_aero_forces_moments_T<double>(aerodynamic_properties, structural_properties, twist, static_atm_state, u, windB);

        return { dynamics::Force{ loads.F }, dynamics::Moment{ loads.M } };
    }

    AerodynamicState compute_aerodynamic_state(const dynamics::RigidBodyState& rigid_body_state, const atmospheric::Wind& windB) {
        const dynamics::Twist_T<double> twist{ .v = rigid_body_state.v.data, .w = rigid_body_state.w.data };

        const AerodynamicState_T<double> ads = compute_aerodynamic_state_T<double>(twist, windB);

        return { FreeStreamVelocity{ ads.Vinf }, AngleOfAttack{ ads.alpha }, SideslipAngle{ ads.beta } };
    }

    dynamics::OrientationMatrix CBS(const aerodynamics::AngleOfAttack& alpha) {
        Eigen::Matrix3d CBS;
        double a = alpha.data;
        CBS     <<   util::cos(a),   0,   util::sin(a),
                                0,   1,              0,
                    -util::sin(a),   0,   util::cos(a);
        return { CBS };
    };

    dynamics::OrientationMatrix CSW(const aerodynamics::SideslipAngle& beta) {
        Eigen::Matrix3d CSW;
        double b = beta.data;
        CSW     <<   util::cos(b),   util::sin(b),   0,
                    -util::sin(b),   util::cos(b),   0,
                                0,              0,   1;
        return { CSW };
    };

}
