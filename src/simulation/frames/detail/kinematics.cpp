#include "simulation/frames/private/detail/kinematics.hpp"
#include "simulation/frames/public/detail/kinematics.hpp"
#include "simulation/transforms/public/detail/se3.hpp"

namespace frames {

    Eigen::Vector3d transform_vec(const Eigen::Vector3d& vA, const Frame& A, const Frame& B) {
        Eigen::Matrix3d CAB = CRF(B, A);
        Eigen::Vector3d vB = CAB * vA;
        return vB;
    }

    Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const Frame& A, const Frame& B) {
        Eigen::Matrix4d HAB = HRF(B, A);
        Eigen::Vector3d pB = transforms::apply_H(HAB, pA);
        return pB;
    }

    dynamics::HomogeneousTransformationMatrix H_from_R(const Frame& F, const Frame& R) {
        return { HRF(F, R) };
    }

    std::tuple<dynamics::TranslationalVelocity, dynamics::AngularVelocity> vel_from_R(const Frame& F, const Frame& R) {
        auto [vF_FE, wF_FE] = vel_from_E(F);
        auto [vR_RE, wR_RE] = vel_from_E(R);

        dynamics::HomogeneousTransformationMatrix HRF = H_from_R(F, R);
        Eigen::Matrix3d CRF = HRF.C().data;
        Eigen::Vector3d pR_FR = HRF.p().data;

        Eigen::Vector3d vF_FR = vF_FE.data - CRF * (vR_RE.data + wR_RE.data.cross(pR_FR));
        Eigen::Vector3d wF_FR = wF_FE.data - CRF * wR_RE.data;

        return { { vF_FR }, { wF_FR } };
    }

}
