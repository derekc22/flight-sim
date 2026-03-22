#include <Eigen/Dense>
#include <vector>
#include <string>
#include <unordered_map>
#include <optional>
#include <algorithm>
#include <cmath>
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/structural/structural.hpp"
#include <simulation/dynamics/dynamics.hpp>
#include <simulation/atmospheric/atmospheric.hpp>
#include "simulation/global/global.hpp"

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


    SurfaceKinematics compute_surface_kinematics(
        const Surface& s,
        const structural::StructuralProperties& structuralProperties,
        const dynamics::RigidBodyState& rigidBodyState,
        const atmospheric::Density& rho,
        const atmospheric::Wind& windB
    ) {
        SurfaceKinematics out;

        const Eigen::Vector3d vB = rigidBodyState.v.data;
        const Eigen::Vector3d wB = rigidBodyState.w.data;
        const Eigen::Vector3d CG = structuralProperties.CG.data;

        out.r_ac_B  = s.p_ac - CG;
        out.v_rel_B = (vB - windB.data) + wB.cross(out.r_ac_B);
        out.V = out.v_rel_B.norm();

        if (out.V < global::eps) {
            out.alpha = 0.0;
            out.qbar  = 0.0;
            out.p_hat = 0.0;
            out.q_hat = 0.0;
            out.r_hat = 0.0;
            return out;
        }

        const Eigen::Vector3d n_hat = global::norm(s.n);
        const double arg = global::clamp_to_1(out.v_rel_B.dot(n_hat) / out.V);

        // alpha_i = i_i - asin( (V dot n) / |V| )
        out.alpha = s.i - std::asin(arg);

        out.qbar = 0.5 * rho.data * out.V * out.V;

        out.p_hat = rigidBodyState.w.p() * s.span  / (2.0 * out.V);
        out.q_hat = rigidBodyState.w.q() * s.chord / (2.0 * out.V);
        out.r_hat = rigidBodyState.w.r() * s.span  / (2.0 * out.V);


        return out;
    }


    SurfaceCoefficients compute_surface_coefficients(
        const Surface& s,
        const SurfaceKinematics& sk,
        const control::ControlSurfaceInputs& u,
        const control::ControlProperties& cp
    ) {
        SurfaceCoefficients out;

        const control::ControlSurfaceInputs u_clipped{
            .elevator = global::clamp_symmetric(u.elevator, cp.limits.elevator_max),
            .aileron = global::clamp_symmetric(u.aileron, cp.limits.aileron_max),
            .rudder = global::clamp_symmetric(u.rudder, cp.limits.rudder_max),
            .flap = global::clamp_positive(u.flap, cp.limits.flap_max),
            .spoiler = global::clamp_positive(u.spoiler, cp.limits.spoiler_max),
        };

        const double CLalpha = 2.0 * M_PI * (s.AR / (2.0 + s.AR));

        out.CL.data = s.CL0 + CLalpha * sk.alpha;
        out.CM.data = s.CM0 + s.CMa * sk.alpha;

        double dCD_extra = 0.0;

        // dynamic derivatives
        out.CL.data += s.dyn.CL_phat * sk.p_hat + s.dyn.CL_qhat * sk.q_hat + s.dyn.CL_rhat * sk.r_hat;

        out.CM.data += s.dyn.CM_phat * sk.p_hat + s.dyn.CM_qhat * sk.q_hat + s.dyn.CM_rhat * sk.r_hat;

        dCD_extra += s.dyn.CD_phat * sk.p_hat + s.dyn.CD_qhat * sk.q_hat + s.dyn.CD_rhat * sk.r_hat;

        // control increments
        out.CL.data += s.ctrl.dCL_de * u_clipped.elevator + s.ctrl.dCL_da * u_clipped.aileron + s.ctrl.dCL_dr * u_clipped.rudder + s.ctrl.dCL_df * u_clipped.flap + s.ctrl.dCL_ds * u_clipped.spoiler;

        out.CM.data += s.ctrl.dCM_de * u_clipped.elevator + s.ctrl.dCM_da * u_clipped.aileron + s.ctrl.dCM_dr * u_clipped.rudder + s.ctrl.dCM_df * u_clipped.flap + s.ctrl.dCM_ds * u_clipped.spoiler;

        dCD_extra += s.ctrl.dCD_de * std::abs(u_clipped.elevator) + s.ctrl.dCD_da * std::abs(u_clipped.aileron) + s.ctrl.dCD_dr * std::abs(u_clipped.rudder) + s.ctrl.dCD_df * std::abs(u_clipped.flap) + s.ctrl.dCD_ds * std::abs(u_clipped.spoiler);

        out.CD.data = s.CD0 + s.CDa * (sk.alpha - s.a0) * (sk.alpha - s.a0) + (out.CL.data * out.CL.data) / (M_PI * s.e * s.AR) + dCD_extra;

        return out;
    }

    AerodynamicLoad compute_surface_loads(const Surface& s,const SurfaceKinematics& sk,const SurfaceCoefficients& sc) {
        if (sk.V < global::eps) {
            return AerodynamicLoad{ 
                dynamics::Force{ global::Zero3 }, 
                dynamics::Moment{ global::Zero3 } 
            };
        }

        const Eigen::Vector3d n_hat = global::norm(s.n);

        // drag points opposite local relative velocity
        const Eigen::Vector3d d_hat = -sk.v_rel_B / sk.V;

        // lift direction is surface normal projected into plane normal to drag
        Eigen::Vector3d l_hat = n_hat - n_hat.dot(d_hat) * d_hat;
        l_hat = global::norm(l_hat);

        // moment axis consistent with positive CM convention
        Eigen::Vector3d m_hat = l_hat.cross(d_hat);
        m_hat = global::norm(m_hat);

        const double L = sk.qbar * s.area * sc.CL.data;
        const double D = sk.qbar * s.area * sc.CD.data;
        const double Mmag = sk.qbar * s.area * s.chord * sc.CM.data;

        const Eigen::Vector3d FB = L * l_hat + D * d_hat;
        const Eigen::Vector3d MB = sk.r_ac_B.cross(FB) + Mmag * m_hat;

        return AerodynamicLoad{
            dynamics::Force{ FB },
            dynamics::Moment{ MB }
        };
    }


    AerodynamicLoad step_aero_forces_moments(
        const AerodynamicProperties& aerodynamicProperties,
        const structural::StructuralProperties& structuralProperties,
        const dynamics::RigidBodyState& rigidBodyState,
        const atmospheric::Density& rho,
        const control::ControlSurfaceInputs& u,
        const control::ControlProperties& cp,
        const atmospheric::Wind& windB
    ) {
        Eigen::Vector3d FB = global::Zero3;
        Eigen::Vector3d MB = global::Zero3;

        for (const Surface& s : aerodynamicProperties.surfaces) {
            SurfaceKinematics sk = compute_surface_kinematics(s, structuralProperties, rigidBodyState, rho, windB);

            SurfaceCoefficients sc = compute_surface_coefficients(s, sk, u, cp);
            AerodynamicLoad loads = compute_surface_loads(s, sk, sc);

            FB += loads.F.data;
            MB += loads.M.data;
        }

        return AerodynamicLoad{ dynamics::Force{ FB }, dynamics::Moment{ MB } };
    }






    AerodynamicState compute_aerodynamic_state(const dynamics::RigidBodyState& rigidBodyState, const atmospheric::Wind& windB){
        const Eigen::Vector3d vB_rel = rigidBodyState.v.data - windB.data;

        const double Vinf = vB_rel.norm();

        double alpha = 0.0;
        double beta  = 0.0;

        if (Vinf > global::eps) {
            const double v1 = vB_rel.x();
            const double v2 = vB_rel.y();
            const double v3 = vB_rel.z();

            alpha = std::atan2(v3, v1);

            double arg = global::clamp_to_1(v2 / Vinf);
            beta = std::asin(arg);
        }

        return AerodynamicState{ FreeStreamVelocity{ Vinf }, AngleOfAttack{ alpha }, SideslipAngle{ beta } };
    }



    dynamics::OrientationMatrix CBS(const aerodynamics::AngleOfAttack& alpha) {
        Eigen::Matrix3d CBS;
        const double a = alpha.data;
        CBS     <<   std::cos(a),   0,   std::sin(a),
                               0,   1,             0,
                    -std::sin(a),   0,    std::cos(a);
        return dynamics::OrientationMatrix{ CBS };
    };

    dynamics::OrientationMatrix CSW(const aerodynamics::SideslipAngle& beta) {
        Eigen::Matrix3d CSW;
        const double b = beta.data;
        CSW     <<   std::cos(b),   std::sin(b),   0,
                    -std::sin(b),   std::cos(b),   0,
                               0,             0,   1;
        return dynamics::OrientationMatrix{ CSW };
    };


};
