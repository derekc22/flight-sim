#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Core>
#include "simulation/actuators/surface/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/frames/public.hpp"
#include "simulation/util/trig/public.hpp"

namespace aerodynamics {

    AerodynamicsManager::AerodynamicsManager(std::vector<Surface> s) : surfaces(std::move(s)) {
        compute_surface_geometry();
    }

    void AerodynamicsManager::compute_surface_geometry() {
        for (Surface& s : surfaces) {
            s.area = s.chord * s.span;
            s.AR   = s.span / s.chord;
        }
    }

    AerodynamicState compute_aerodynamic_state(const frames::Frame& F, const frames::Frame& R, const atmospheric::Wind& windB) {
        return compute_aerodynamic_state(dynamics::compute_rigid_body_state(F, R), windB);
    }

    dynamics::Wrench step_aero_forces_moments(const AerodynamicsManager& aerodynamics_manager, const structural::CenterOfGravity& pB_GB, const dynamics::RigidBodyState& X, const atmospheric::StaticAtmosphericState& atm, const actuators::SurfaceActuatorInputs_T<double>& u, const atmospheric::Wind& windB) {
        const dynamics::Twist_T<double> twist{
            .v = X.v.data,
            .w = X.w.data,
        };

        const dynamics::Wrench_T<double> loads = step_aero_forces_moments_T<double>(aerodynamics_manager, pB_GB.data, twist, atm, u, windB);

        return { dynamics::Force{ loads.F }, dynamics::Moment{ loads.M } };
    }

    AerodynamicState compute_aerodynamic_state(const dynamics::RigidBodyState& X, const atmospheric::Wind& windB) {
        const dynamics::Twist_T<double> twist{ .v = X.v.data, .w = X.w.data };

        const AerodynamicState_T<double> aero = compute_aerodynamic_state_T<double>(twist, windB);

        return { FreeStreamVelocity{ aero.Vinf }, AngleOfAttack{ aero.alpha }, SideslipAngle{ aero.beta } };
    }

    dynamics::OrientationMatrix CBS(const AngleOfAttack& alpha) {
        Eigen::Matrix3d CBS;
        double a = alpha.data;
        CBS     <<   util::cos(a),   0,   util::sin(a),
                                0,   1,              0,
                    -util::sin(a),   0,   util::cos(a);
        return { CBS };
    };

    dynamics::OrientationMatrix CSW(const SideslipAngle& beta) {
        Eigen::Matrix3d CSW;
        double b = beta.data;
        CSW     <<   util::cos(b),   util::sin(b),   0,
                    -util::sin(b),   util::cos(b),   0,
                                0,              0,   1;
        return { CSW };
    };

}
