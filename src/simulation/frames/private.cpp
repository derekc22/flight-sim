#include "simulation/frames/private.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/transforms/public.hpp"

namespace frames {

    Eigen::Matrix3d CRF(const Frame& F) {
        Eigen::Matrix3d C = Eigen::Matrix3d::Identity();
        const Frame* pF = &F;
        while (pF != nullptr){
            C *= pF->view().H->C().data;
            pF = pF->parent;
        }
        return C;
    }

    Eigen::Vector3d pRF(const Frame& F) {
        Eigen::Vector3d p = constants::Zero3;
        const Frame* pF = &F;
        while (pF != nullptr) {
            const FrameView fv = pF->view();
            p = fv.H->C().data.transpose() * p + fv.H->p().data;
            pF = pF->parent;
        }
        return p;
    }

    Eigen::Matrix4d HRF(const Frame& F) {
        return transforms::make_HC(CRF(F), pRF(F), "translate");
    }

}
