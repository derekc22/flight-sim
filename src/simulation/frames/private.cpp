#include <tuple>
#include "simulation/dynamics/public.hpp"
#include "simulation/frames/private.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/transforms/public.hpp"

namespace frames {

    Eigen::Matrix3d CRF(const Frame& F, const Frame& R) {
        return transforms::C_from_H(HRF(F, R));
    }

    Eigen::Vector3d pRF(const Frame& F, const Frame& R) {
        return transforms::p_from_H(HRF(F, R));
    }

    Eigen::Matrix4d HRF(const Frame& F, const Frame& R) {
        Eigen::Matrix4d HEF = constants::HI;
        const Frame* pF = &F;
        while (pF->parent != nullptr) {
            HEF *= pF->view().H->data;
            pF = pF->parent;
        }

        Eigen::Matrix4d HER = constants::HI;
        const Frame* pR = &R;
        while (pR->parent != nullptr) {
            HER *= pR->view().H->data;
            pR = pR->parent;
        }

        return HEF * transforms::make_Hinv(HER);
    }

    std::tuple<dynamics::TranslationalVelocity, dynamics::AngularVelocity> vel_from_E(const Frame& F) {
        const Frame* pF = &F;

        Eigen::Vector3d p_FNext = constants::Zero3;
        Eigen::Vector3d v_FNextNext = constants::Zero3;
        Eigen::Vector3d w_FNextNext = constants::Zero3;

        while (pF->parent != nullptr) {
            const FrameView fv = pF->view();

            const Frame* pNext = pF->parent;
            const FrameView next_fv = pNext->view();
            Eigen::Vector3d v_NextNextNext = next_fv.v->data;
            Eigen::Vector3d w_NextNextNext = next_fv.w->data;

            Eigen::Matrix3d CCurr_Next = fv.H->C().data.transpose();

            p_FNext = fv.H->p().data + CCurr_Next * p_FNext;

            if (pF == &F) {
                v_FNextNext = fv.v->data;
                w_FNextNext = fv.w->data;
            }

            v_FNextNext = v_NextNextNext + CCurr_Next * v_FNextNext + w_NextNextNext.cross(p_FNext);
            w_FNextNext = w_NextNextNext + CCurr_Next * w_FNextNext;

            pF = pNext;
        }

        Eigen::Matrix3d CEF = CRF(F, *pF);
        return { { CEF * v_FNextNext }, { CEF * w_FNextNext } };
    }

}
