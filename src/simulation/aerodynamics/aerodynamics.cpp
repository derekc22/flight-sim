#include <Eigen/Dense>
#include <vector>
#include <string>
#include <unordered_map>
#include <iostream>
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/structural/structural.hpp"
#include <simulation/dynamics/dynamics.hpp>
#include <simulation/atmospheric/atmospheric.hpp>

namespace aerodynamics {

    AerodynamicProperties::AerodynamicProperties(std::vector<Surface> s) : surfaces(s) {
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
        for (size_t i = 0; i < surfaces.size(); ++i){
            m[surfaces[i].id] = i;
        }
        return m;
    }


    dynamics::Wrench step_aero_forces_moments(
        const AerodynamicProperties& aerodynamicProperties, 
        const structural::StructuralProperties& structuralProperties, 
        const dynamics::RigidBodyState& rigidBodyState, 
        const atmospheric::Density& rho
    ) {
        Eigen::Vector3d FB = global::Zero3;
        Eigen::Vector3d MB = global::Zero3;

        const Eigen::Vector3d vB = rigidBodyState.v.data;     // no wind: V_B_BA = V_B_BE
        const Eigen::Vector3d wB = rigidBodyState.w.data;
        const Eigen::Vector3d CG = structuralProperties.CG.data;

        for (const Surface& s : aerodynamicProperties.surfaces) {
            const Eigen::Vector3d r_ac = s.p_ac - CG;          // CG -> AC
            const Eigen::Vector3d v_rel = vB + wB.cross(r_ac); // include rotational component

            const double V = v_rel.norm();
            if (V < 1e-9) { continue; }

            const double q = 0.5 * rho.data * V * V;

            const Eigen::Vector3d n_hat = (s.n.norm() > 1e-12) ? (s.n.normalized()) : global::Zero3;

            // alpha^{S_i} = i^{S_i} - asin( (V dot n) / ||V|| )
            double arg = v_rel.dot(n_hat) / V;
            arg = std::max(-1.0, std::min(1.0, arg));
            const double alpha = s.i - std::asin(arg);

            // C_{Lalpha}^{S_i} = 2*pi * AR/(2+AR)
            const double CLalpha = 2.0 * M_PI * (s.AR / (2.0 + s.AR));

            const double CL = s.CL0 + CLalpha * alpha;
            const double CD = s.CD0 + s.CDa * (alpha - s.a0) * (alpha - s.a0) + (CL * CL) / (M_PI * s.e * s.AR);
            const double CM = s.CM0 + s.CMa * alpha;

            const double L = q * s.area * CL;
            const double D = q * s.area * CD;
            const double Mmag = q * s.area * s.chord * CM;

            const Eigen::Vector3d d_hat = -v_rel / V; // drag along -V

            // lift direction: component of n orthogonal to drag, normalized
            Eigen::Vector3d l_dir = n_hat - (n_hat.dot(d_hat)) * d_hat;
            const double ln = l_dir.norm();
            if (ln > 1e-12) { l_dir /= ln; } else { l_dir.setZero(); }

            const Eigen::Vector3d Fi = L * l_dir + D * d_hat;

            // pitching moment axis (perp to plane of drag and lift)
            // Eigen::Vector3d m_hat = d_hat.cross(l_dir); // <- THIS IS WRONG
            Eigen::Vector3d m_hat = l_dir.cross(d_hat); // <- THIS IS RIGHT
            const double mn = m_hat.norm();
            if (mn > 1e-12) { m_hat /= mn; } else { m_hat.setZero(); }

            Eigen::Vector3d Mi = r_ac.cross(Fi) + Mmag * m_hat;

            FB += Fi;
            MB += Mi;
        }

        return dynamics::Wrench{ dynamics::Force{ FB }, dynamics::Moment{ MB } };
    }






    AerodynamicState compute_aerodynamic_state(const dynamics::RigidBodyState& rigidBodyState){
        const Eigen::Vector3d& vB = rigidBodyState.v.data;

        const double Vinf = vB.norm();

        double alpha = 0.0;
        double beta  = 0.0;

        if (Vinf > 1e-9) {
            const double v1 = vB.x();
            const double v2 = vB.y();
            const double v3 = vB.z();

            alpha = std::atan2(v3, v1);

            double arg = v2 / Vinf;
            arg = std::max(-1.0, std::min(1.0, arg));
            beta = std::asin(arg);
        }

        return AerodynamicState{ Vinf, alpha, beta };
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
