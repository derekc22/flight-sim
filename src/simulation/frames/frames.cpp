#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include <algorithm>
#include "simulation/frames/frames.hpp"
#include "simulation/transforms/transforms.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/geography/geography.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"

namespace frames {

    MutableFrameView NEDFrameECEF::view() {
        return { &HEN, &qEN, &eulEN, &CEN_dot, &qEN_dot, &wN_NE, &eulEN_dot, &wq_NE, &vN_NE, &gN };
    }
    FrameView NEDFrameECEF::view() const {
        return { &HEN, &qEN, &eulEN, &CEN_dot, &qEN_dot, &wN_NE, &eulEN_dot, &wq_NE, &vN_NE, &gN };
    }

    MutableFrameView FRDFrameECEF::view() {
        return { &HEB, &qEB, &eulEB, &CEB_dot, &qEB_dot, &wB_BE, &eulEB_dot, &wq_BE, &vB_BE, &gB };
    }
    FrameView FRDFrameECEF::view() const {
        return { &HEB, &qEB, &eulEB, &CEB_dot, &qEB_dot, &wB_BE, &eulEB_dot, &wq_BE, &vB_BE, &gB };
    }

    MutableFrameView FRDFrameNED::view() {
        return { &HNB, &qNB, &eulNB, &CNB_dot, &qNB_dot, &wB_BN, &eulNB_dot, &wq_BN, &vB_BN, &gB };
    }
    FrameView FRDFrameNED::view() const {
        return { &HNB, &qNB, &eulNB, &CNB_dot, &qNB_dot, &wB_BN, &eulNB_dot, &wq_BN, &vB_BN, &gB };
    }

    MutableFrameView STABFrameFRD::view() {
        return { &HBS, &qBS, &eulBS, &CBS_dot, &qBS_dot, &wS_SB, &eulBS_dot, &wq_SB, &vS_SB, &gS };
    }
    FrameView STABFrameFRD::view() const {
        return { &HBS, &qBS, &eulBS, &CBS_dot, &qBS_dot, &wS_SB, &eulBS_dot, &wq_SB, &vS_SB, &gS };
    }

    MutableFrameView WINDFrameSTAB::view() {
        return { &HSW, &qSW, &eulSW, &CSW_dot, &qSW_dot, &wW_WS, &eulSW_dot, &wq_WS, &vW_WS, &gW };
    }
    FrameView WINDFrameSTAB::view() const {
        return { &HSW, &qSW, &eulSW, &CSW_dot, &qSW_dot, &wW_WS, &eulSW_dot, &wq_WS, &vW_WS, &gW };
    }








    void Frame::_set(const dynamics::homogeneousFrameTransformationMatrix& H){
        _set(H.C());
        _set(H.p());
    }
    void Frame::_set(const dynamics::OrientationMatrix& C){
        MutableFrameView mfv = view();
        mfv.H->set(C);
        mfv.q->set(C);
        mfv.eul->set(C);
    }
    void Frame::_set(const dynamics::Position& p){
        MutableFrameView mfv = view();
        mfv.H->set(p);
    }
    void Frame::_set(const dynamics::OrientationQuaternion& q){
        MutableFrameView mfv = view();
        dynamics::OrientationQuaternion q_{ transforms::normalize_and_canonicalize(q.data) };
        *mfv.q = q_;
        mfv.H->set(q_);
        mfv.eul->set(q_);
    }
    void Frame::_set(const dynamics::EulerAngles& eul){
        MutableFrameView mfv = view();
        mfv.H->set(eul);
        mfv.q->set(eul);
        *mfv.eul = eul;
    }
    void Frame::_set(const dynamics::OrientationMatrixRate& C_dot){
        MutableFrameView mfv = view();
        *mfv.C_dot = C_dot;
        dynamics::AngularVelocity w = dynamics::_CIB_dot_to_wB_BI(*mfv.C_dot, mfv.H->C());
        mfv.q_dot->set(*mfv.q, w);
        *mfv.w = w;
        mfv.eul_dot->set(w, *mfv.eul);
        mfv.wq->set(w);
    }
    void Frame::_set(const dynamics::OrientationQuaternionRate& q_dot){
        MutableFrameView mfv = view();
        mfv.C_dot->set(q_dot, *mfv.q, mfv.H->C());
        *mfv.q_dot = q_dot;
        dynamics::AngularVelocity w = dynamics::_qIB_dot_to_wB_BI(q_dot, *mfv.q);
        *mfv.w = w;
        mfv.eul_dot->set(w, *mfv.eul);
        mfv.wq->set(w);
    }
    void Frame::_set(const dynamics::AngularVelocity& w){
        MutableFrameView mfv = view();
        mfv.C_dot->set(mfv.H->C(), w);
        mfv.q_dot->set(*mfv.q, w);
        *mfv.w = w;
        mfv.eul_dot->set(w, *mfv.eul);
        mfv.wq->set(w);
    }
    void Frame::_set(const dynamics::EulerAngleRates& eul_dot){
        MutableFrameView mfv = view();
        dynamics::AngularVelocity w = dynamics::_eul_dot_to_wB_BI(eul_dot, *mfv.eul);
        mfv.C_dot->set(mfv.H->C(), w);
        mfv.q_dot->set(*mfv.q, w);
        *mfv.w = w;
        *mfv.eul_dot = eul_dot;
        mfv.wq->set(w);
    }
    void Frame::_set(const dynamics::AngularVelocityQuaternion& wq){
        MutableFrameView mfv = view();
        dynamics::AngularVelocity w = wq.w();
        mfv.C_dot->set(mfv.H->C(), w);
        mfv.q_dot->set(*mfv.q, w);
        *mfv.w = w;
        mfv.eul_dot->set(w, *mfv.eul);
        *mfv.wq = wq;
    }
    void Frame::_set(const dynamics::LinearVelocity& v){
        MutableFrameView mfv = view();
        *mfv.v = v;
    }
    void Frame::_set(const dynamics::Gravity& g){
        MutableFrameView mfv = view();
        *mfv.g = g;
    }

    void Frame::set(const SetOptions& opts){
        if (opts.H.has_value())         { _set(*opts.H); }
        if (opts.C.has_value())         { _set(*opts.C); }
        if (opts.p.has_value())         { _set(*opts.p); }
        if (opts.q.has_value())         { _set(*opts.q); }
        if (opts.eul.has_value())       { _set(*opts.eul); }
        if (opts.C_dot.has_value())     { _set(*opts.C_dot); }
        if (opts.q_dot.has_value())     { _set(*opts.q_dot); }
        if (opts.w.has_value())         { _set(*opts.w); }
        if (opts.eul_dot.has_value())   { _set(*opts.eul_dot); }
        if (opts.wq.has_value())        { _set(*opts.wq); }
        if (opts.v.has_value())         { _set(*opts.v); }
        // if (opts.a.has_value())         { _set(*opts.a); }
        if (opts.g.has_value())         { _set(*opts.g); }
        // opts.clear();
    }

    /** @deprecated */
    // void SetOptions::clear() noexcept { *this = SetOptions{}; }


    Frame::Frame(std::string n, Frame* p) : name(n), parent(p) {};

    Frame::~Frame() {
        // Destructor for frame A

        // Parent side cleanup (for each parent P in dependent_on)
        for (auto it = dependent_on.begin(); it != dependent_on.end(); ) {
            Frame* parent = *it;
            parent->dependents.erase(this); // Remove A from P’s dependents
            it = dependent_on.erase(it);    // Remove P from A’s dependent_on
        }
        // Child side (for each child C in dependents):
        for (auto it = dependents.begin(); it != dependents.end(); ) {
            Frame* dep = *it;
            dep->dependent_on.erase(this);  // Remove A from C’s dependent_on
            it = dependents.erase(it);      // Remove C from A’s dependents
        }
    }


    void Frame::_add_as_direct_dependent(Frame* p) {
        dependents.insert(p);
        p->dependent_on.insert(this);
    }

    // interpret nullptr parent as ECEFFrame
    NEDFrameECEF::NEDFrameECEF() : Frame("NEDFrameECEF", nullptr) {};
    FRDFrameECEF::FRDFrameECEF() : Frame("FRDFrameECEF", nullptr) {};
    FRDFrameNED::FRDFrameNED(NEDFrameECEF* pNEDFrameECEF) : Frame("FRDFrameNED", pNEDFrameECEF) {};
    STABFrameFRD::STABFrameFRD(FRDFrameNED* pFRDFrameNED) : Frame("STABFrameFRD", pFRDFrameNED) {};
    WINDFrameSTAB::WINDFrameSTAB(STABFrameFRD* pSTABFrameFRD) : Frame("WINDFrameSTAB", pSTABFrameFRD) {};




    Eigen::Matrix3d CRF(const Frame& F) {
        Eigen::Matrix3d C = Eigen::Matrix3d::Identity();
        const Frame* pF = &F;
        while (pF != nullptr){
            C *= pF->view().H->C().data;
            pF = pF->parent;
        }
        return C ;
    }

    Eigen::Vector3d transform_vec(const Eigen::Vector3d& vA, const Frame& A, const Frame& B) {
        Eigen::Matrix3d CRA = CRF(A);
        Eigen::Matrix3d CRB = CRF(B);
        Eigen::Vector3d vB = CRB * CRA.transpose() * vA;
        return vB;
    }

    Eigen::Vector3d transform_vec(const Eigen::Vector3d& vA, const Frame& A, const ECEFFrame&) {
        Eigen::Matrix3d CRA = CRF(A);
        Eigen::Vector3d vB = CRA.transpose() * vA;
        return vB;
    }

    Eigen::Vector3d transform_vec(const Eigen::Vector3d& vA, const ECEFFrame&, const Frame& B) {
        Eigen::Matrix3d CRB = CRF(B);
        Eigen::Vector3d vB = CRB * vA;
        return vB;
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

    Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const Frame& A, const Frame& B) {
        Eigen::Matrix4d HRA = HRF(A);
        Eigen::Matrix4d HRB = HRF(B);
        Eigen::Vector3d pB = transforms::apply_H(HRB * transforms::make_Hinv(HRA), pA);
        return pB;
    }

    Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const Frame& A, const ECEFFrame&) {
        Eigen::Matrix4d HRA = HRF(A);
        Eigen::Vector3d pB = transforms::apply_H(transforms::make_Hinv(HRA), pA);
        return pB;
    }

    Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const ECEFFrame&, const Frame& B) {
        Eigen::Matrix4d HRB = HRF(B);
        Eigen::Vector3d pB = transforms::apply_H(HRB, pA);
        return pB;
    }


}
