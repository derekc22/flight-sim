#include <Eigen/Dense>
#include <vector>
#include <string>
#include <unordered_map>
#include <optional>
#include <algorithm>
#include <cmath>
#include <format>
#include <stdexcept>
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/frames/frames.hpp"
#include <simulation/dynamics/dynamics.hpp>
#include <simulation/atmospheric/atmospheric.hpp>
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"

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

    AerodynamicState aerodynamic_state(const frames::Frame& F, const atmospheric::Wind& windB) {
        if (F.parent != nullptr && F.parent->name != "NEDFrameECEF") {
            throw std::invalid_argument(std::format("aerodynamics::aerodynamic_state: Invalid frame input, the parent of {} must be an inertial frame: ECEFFrame or NEDFrameECEF", F.name));
        }
        return aerodynamics::compute_aerodynamic_state(dynamics::rigid_body_state(F), windB);
    }

    std::unordered_map<std::string, size_t> AerodynamicProperties::build_IDs() {
        std::unordered_map<std::string, size_t> m;
        for (size_t i = 0; i < surfaces.size(); ++i) {
            m[surfaces[i].id] = i;
        }
        return m;
    }


    SurfaceKinematics compute_surface_kinematics(
        const Surface& s,
        const structural::StructuralProperties& structural_properties,
        const dynamics::RigidBodyState& rigid_body_state,
        const atmospheric::AirDensity& rho,
        const atmospheric::Wind& windB
    ) {
        const dynamics::Twist_T<double> twist{
            .v = rigid_body_state.v.data,
            .w = rigid_body_state.w.data,
        };

        const SurfaceKinematics_T<double> sk = compute_surface_kinematics_T<double>(s, structural_properties, twist, rho, windB);

        return {
            .r_ac_B = sk.r_ac_B,
            .v_rel_B = sk.v_rel_B,
            .V = sk.V,
            .qbar = sk.qbar,
            .alpha = sk.alpha,
            .p_hat = sk.p_hat,
            .q_hat = sk.q_hat,
            .r_hat = sk.r_hat,
        };
    }


    SurfaceCoefficients compute_surface_coefficients(const Surface& s, const SurfaceKinematics& sk, const control::ControlSurfaceInputs& u, const control::ControlProperties& cp) {
        const SurfaceCoefficients_T<double> sc = compute_surface_coefficients_T<double>(
            s,
            SurfaceKinematics_T<double>{
                .r_ac_B = sk.r_ac_B,
                .v_rel_B = sk.v_rel_B,
                .V = sk.V,
                .qbar = sk.qbar,
                .alpha = sk.alpha,
                .p_hat = sk.p_hat,
                .q_hat = sk.q_hat,
                .r_hat = sk.r_hat,
            },
            ControlSurfaceInputs_T<double>{
                .elevator = u.elevator,
                .aileron = u.aileron,
                .rudder = u.rudder,
                .flap = u.flap,
                .spoiler = u.spoiler,
            },
            cp
        );

        return {
            .CL = LiftCoefficient{ sc.CL },
            .CD = DragCoefficient{ sc.CD },
            .CM = MomentCoefficient{ sc.CM },
        };
    }

    AerodynamicLoad compute_surface_loads(const Surface& s, const SurfaceKinematics& sk, const SurfaceCoefficients& sc) {
        const AerodynamicLoad_T<double> loads = compute_surface_loads_T<double>(
            s,
            SurfaceKinematics_T<double>{
                .r_ac_B = sk.r_ac_B,
                .v_rel_B = sk.v_rel_B,
                .V = sk.V,
                .qbar = sk.qbar,
                .alpha = sk.alpha,
                .p_hat = sk.p_hat,
                .q_hat = sk.q_hat,
                .r_hat = sk.r_hat,
            },
            SurfaceCoefficients_T<double>{
                .CL = sc.CL.data,
                .CD = sc.CD.data,
                .CM = sc.CM.data,
            }
        );

        return { dynamics::Force{ loads.F }, dynamics::Moment{ loads.M } };
    }


    AerodynamicLoad step_aero_forces_moments(
        const AerodynamicProperties& aerodynamic_properties,
        const structural::StructuralProperties& structural_properties,
        const dynamics::RigidBodyState& rigid_body_state,
        const atmospheric::AirDensity& rho,
        const control::ControlSurfaceInputs& u,
        const control::ControlProperties& cp,
        const atmospheric::Wind& windB
    ) {
        const dynamics::Twist_T<double> twist{
            .v = rigid_body_state.v.data,
            .w = rigid_body_state.w.data,
        };

        const AerodynamicLoad_T<double> loads = step_aero_forces_moments_T<double>(
            aerodynamic_properties,
            structural_properties,
            twist,
            rho,
            ControlSurfaceInputs_T<double>{
                .elevator = u.elevator,
                .aileron = u.aileron,
                .rudder = u.rudder,
                .flap = u.flap,
                .spoiler = u.spoiler,
            },
            cp,
            windB
        );

        return { dynamics::Force{ loads.F }, dynamics::Moment{ loads.M } };
    }






    AerodynamicState compute_aerodynamic_state(const dynamics::RigidBodyState& rigid_body_state, const atmospheric::Wind& windB) {
        const dynamics::Twist_T<double> twist{ .v = rigid_body_state.v.data, .w = rigid_body_state.w.data };

        const AerodynamicState_T<double> ads = compute_aerodynamic_state_T<double>(twist, windB);

        return { FreeStreamVelocity{ ads.Vinf }, AngleOfAttack{ ads.alpha }, SideslipAngle{ ads.beta } };
    }



    dynamics::OrientationMatrix CBS(const aerodynamics::AngleOfAttack& alpha) {
        Eigen::Matrix3d CBS;
        const double a = alpha.data;
        CBS     <<   std::cos(a),   0,   std::sin(a),
                               0,   1,             0,
                    -std::sin(a),   0,    std::cos(a);
        return { CBS };
    };

    dynamics::OrientationMatrix CSW(const aerodynamics::SideslipAngle& beta) {
        Eigen::Matrix3d CSW;
        const double b = beta.data;
        CSW     <<   std::cos(b),   std::sin(b),   0,
                    -std::sin(b),   std::cos(b),   0,
                               0,             0,   1;
        return { CSW };
    };


};
