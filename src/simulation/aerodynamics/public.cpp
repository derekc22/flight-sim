#include <cmath>
#include <format>
#include <stdexcept>
#include <utility>
#include <Eigen/Dense>
#include "simulation/actuators/surface/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/frames/public.hpp"
#include "simulation/structural/public.hpp"
#include "simulation/util/public.hpp"

namespace aerodynamics {

    AerodynamicProperties::AerodynamicProperties(std::vector<Surface> s) : surfaces(std::move(s)) {
        compute_aero_properties();
        surfaceIDs = build_IDs();
    }

    void AerodynamicProperties::compute_aero_properties() {
        for (Surface& s : surfaces) {
            s.area = s.chord * s.span;
            s.AR   = s.span / s.chord;
            s.p_ac = s.p_ref;   // quarter-chord assumed already in p_ref
        }
    }

    std::unordered_map<std::string, size_t> AerodynamicProperties::build_IDs() {
        std::unordered_map<std::string, size_t> m;
        for (size_t i = 0; i < surfaces.size(); ++i) {
            m[surfaces[i].id] = i;
        }
        return m;
    }

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
