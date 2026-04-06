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

    // Eigen::Matrix4d HRF(const ECEFFrame&) {
    //     return constants::HI;
    // }

    Eigen::Matrix4d HRF(const Frame& F) {
        return transforms::make_HC(CRF(F), pRF(F), "translate");
    }


    /** @deprecated */
    // dynamics::OrientationMatrix NEDFrameECEF::CEN_from_lat_lon(const geography::Latitude& lat, const geography::Longitude& lon) {
    //     Eigen::Matrix3d CEN;
    //     double _lat = lat.data;
    //     double _lon = lon.data;
    //     CEN     <<   -std::sin(_lat) * std::cos(_lon),  -std::sin(_lat) * std::sin(_lon),   std::cos(_lat),
    //                                   -std::sin(_lon),                    std::cos(_lon),                0,
    //                  -std::cos(_lat) * std::cos(_lon),  -std::cos(_lat) * std::sin(_lon),  -std::sin(_lat);
    //     return dynamics::OrientationMatrix{ CEN };
    // };

    /** @deprecated */
    // geography::GeographicState ECEFFrame::lat_lon_alt_from_pE(const dynamics::Position & pE) {
    //     double x = pE(0);
    //     double y = pE(1);
    //     double z = pE(2);

    //     double p = std::sqrt(x * x + y * y);
    //     double r = std::sqrt(x * x + y * y + z * z);

    //     // Longitude (radians)
    //     Longitude lon{ std::atan2(y, x) };

    //     // Geocentric latitude (radians)
    //     Latitude lat{ std::atan2(z, p) };

    //     // Altitude above the spherical Earth (meters)
    //     Altitude alt{ r - constants::r_earth };

    //     return geography::GeographicState{ lat, lon, alt };
    // }

    /** @deprecated */
    // dynamics::Position ECEFFrame::pE_from_lat_lon_alt(const geography::GeographicState& geographicState) {
    //     double lat = geographicState.lat.data;   // radians
    //     double lon = geographicState.lon.data;   // radians
    //     double alt = geographicState.alt.data;   // meters

    //     double r = constants::r_earth + alt;

    //     double cos_lat = std::cos(lat);
    //     double sin_lat = std::sin(lat);
    //     double cos_lon = std::cos(lon);
    //     double sin_lon = std::sin(lon);

    //     double x = r * cos_lat * cos_lon;
    //     double y = r * cos_lat * sin_lon;
    //     double z = r * sin_lat;

    //     return dynamics::Position{ x, y, z };
    // }

    /** @deprecated */
    // dynamics::RigidBodyState Frame::RigidBodyState(){
    //     FrameView fv = view();
    //     return dynamics::RigidBodyState { 
    //         .p = fv.H->p(), 
    //         .v = *fv.v, .q = *fv.q, .w = *fv.w
    //     };
    // }

    /** @deprecated */
    // dynamics::RigidBodyState FRDFrameECEF::RigidBodyState(){
    //     return dynamics::RigidBodyState { 
    //         dynamics::RigidBodyState{
    //             .p = HEB.p(), 
    //             .v = vB_BE, .q = qEB, .w = wB_BE
    //         }
    //     };
    // }


    /** @deprecated */
    // MutableFrameView GenericFrame::view() {
    //     // return { &HYX, &qYX, &eulYX, &CYX_dot, &qYX_dot, &wX_XY, &eulYX_dot, &wq_XY, &vX_XY, &aX_XY, &gX };
    //     return { &HYX, &qYX, &eulYX, &CYX_dot, &qYX_dot, &wX_XY, &eulYX_dot, &wq_XY, &vX_XY, &gX };
    // }
    // FrameView GenericFrame::view() const {
    //     return const_cast<GenericFrame*>(this)->view();
    // }

    MutableFrameView NEDFrameECEF::view() {
        // return { &HEN, &qEN, &eulEN, &CEN_dot, &qEN_dot, &wN_NE, &eulEN_dot, &wq_NE, &vN_NE, &aN_NE, &gN };
        return { &HEN, &qEN, &eulEN, &CEN_dot, &qEN_dot, &wN_NE, &eulEN_dot, &wq_NE, &vN_NE, &gN };
    }
    FrameView NEDFrameECEF::view() const {
        return { &HEN, &qEN, &eulEN, &CEN_dot, &qEN_dot, &wN_NE, &eulEN_dot, &wq_NE, &vN_NE, &gN };
    }
    // FrameView NEDFrameECEF::view() const {
    //     return const_cast<NEDFrameECEF*>(this)->view();
    // }

    MutableFrameView FRDFrameECEF::view() {
        // return { &HEB, &qEB, &eulEB, &CEB_dot, &qEB_dot, &wB_BE, &eulEB_dot, &wq_BE, &vB_BE, &aB_BE, &gB };
        return { &HEB, &qEB, &eulEB, &CEB_dot, &qEB_dot, &wB_BE, &eulEB_dot, &wq_BE, &vB_BE, &gB };
    }
    FrameView FRDFrameECEF::view() const {
        return { &HEB, &qEB, &eulEB, &CEB_dot, &qEB_dot, &wB_BE, &eulEB_dot, &wq_BE, &vB_BE, &gB };
    }
    // FrameView FRDFrameECEF::view() const {
    //     return const_cast<FRDFrameECEF*>(this)->view();
    // }

    MutableFrameView FRDFrameNED::view() {
        // return { &HNB, &qNB, &eulNB, &CNB_dot, &qNB_dot, &wB_BN, &eulNB_dot, &wq_BN, &vB_BN, &aB_BN, &gB };
        return { &HNB, &qNB, &eulNB, &CNB_dot, &qNB_dot, &wB_BN, &eulNB_dot, &wq_BN, &vB_BN, &gB };
    }
    FrameView FRDFrameNED::view() const {
        return { &HNB, &qNB, &eulNB, &CNB_dot, &qNB_dot, &wB_BN, &eulNB_dot, &wq_BN, &vB_BN, &gB };
    }
    // FrameView FRDFrameNED::view() const {
    //     return const_cast<FRDFrameNED*>(this)->view();
    // }

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








    void Frame::_set(const dynamics::HomogenousFrameTransformationMatrix& H){
        _set(H.C());
        _set(H.p());
    }
    void Frame::_set(const dynamics::OrientationMatrix& C){
        MutableFrameView mfv = view();
        mfv.H->set(C);
        mfv.q->set(C); // *mfv.q = dynamics::OrientationQuaternion{ transforms::rot_to_quat(C.data) };
        mfv.eul->set(C); // *mfv.eul = dynamics::EulerAngles{ transforms::_C_to_eul_intr(C.data, "ZYX") };
    }
    void Frame::_set(const dynamics::Position& p){
        MutableFrameView mfv = view();
        mfv.H->set(p);
    }
    void Frame::_set(const dynamics::OrientationQuaternion& q){
        MutableFrameView mfv = view();
        dynamics::OrientationQuaternion q_{ transforms::normalize_and_canonicalize(q.data) };
        *mfv.q = q_;
        mfv.H->set(q_); // mfv.H->set(transforms::quat_to_rot(q.data));
        mfv.eul->set(q_); // *mfv.eul = dynamics::EulerAngles{ transforms::_quatC_to_eul_intr(q.data, "ZYX") };
    }
    void Frame::_set(const dynamics::EulerAngles& eul){
        MutableFrameView mfv = view();
        mfv.H->set(eul); // mfv.H->set(transforms::_eul_to_C_intr(eul.psi(), eul.theta(), eul.phi(), "ZYX"));
        mfv.q->set(eul); // *mfv.q = dynamics::OrientationQuaternion{ transforms::_eul_to_quatC_intr(eul.psi(), eul.theta(), eul.phi(), "ZYX") };
        *mfv.eul = eul;
    }
    void Frame::_set(const dynamics::OrientationMatrixRate& C_dot){
        MutableFrameView mfv = view();
        *mfv.C_dot = C_dot;
        dynamics::AngularVelocity w = dynamics::_CIB_dot_to_wB_BI(*mfv.C_dot, mfv.H->C());
        mfv.q_dot->set(*mfv.q, w); // *mfv.q_dot = dynamics::_quat_kin_vel(*mfv.q, w);
        // mfv.q_dot->set(C_dot, mfv.H->C(), *mfv.q); // *mfv.q_dot = dynamics::_CIB_dot_to_qIB_dot(C_dot, mfv.H->C(), *mfv.q);
        *mfv.w = w;
        mfv.eul_dot->set(w, *mfv.eul); // *mfv.eul_dot = dynamics::_wB_BI_to_eul_dot(w, *mfv.eul);
        mfv.wq->set(w);
    }
    void Frame::_set(const dynamics::OrientationQuaternionRate& q_dot){
        MutableFrameView mfv = view();
        mfv.C_dot->set(q_dot, *mfv.q, mfv.H->C()); // *mfv.C_dot = dynamics::_qIB_dot_to_CIB_dot(q_dot, *mfv.q, mfv.H->C());
        *mfv.q_dot = q_dot;
        dynamics::AngularVelocity w = dynamics::_qIB_dot_to_wB_BI(q_dot, *mfv.q);
        *mfv.w = w;
        mfv.eul_dot->set(w, *mfv.eul); // *mfv.eul_dot = dynamics::_wB_BI_to_eul_dot(w, *mfv.eul);
        mfv.wq->set(w);
    }
    void Frame::_set(const dynamics::AngularVelocity& w){
        MutableFrameView mfv = view();
        mfv.C_dot->set(mfv.H->C(), w); // *mfv.C_dot = dynamics::_ddt_CIB(mfv.H->C(), w);
        mfv.q_dot->set(*mfv.q, w); // *mfv.q_dot = dynamics::_quat_kin_vel(*mfv.q, w);
        *mfv.w = w;
        mfv.eul_dot->set(w, *mfv.eul); // *mfv.eul_dot = dynamics::_wB_BI_to_eul_dot(w, *mfv.eul);
        mfv.wq->set(w);
    }
    void Frame::_set(const dynamics::EulerAngleRates& eul_dot){
        MutableFrameView mfv = view();
        dynamics::AngularVelocity w = dynamics::_eul_dot_to_wB_BI(eul_dot, *mfv.eul);
        mfv.C_dot->set(mfv.H->C(), w); // *mfv.C_dot = dynamics::_ddt_CIB(mfv.H->C(), w);
        mfv.q_dot->set(*mfv.q, w); // *mfv.q_dot = dynamics::_quat_kin_vel(*mfv.q, w);
        *mfv.w = w;
        *mfv.eul_dot = eul_dot;
        mfv.wq->set(w);
    }
    void Frame::_set(const dynamics::AngularVelocityQuaternion& wq){
        MutableFrameView mfv = view();
        dynamics::AngularVelocity w = wq.w();
        mfv.C_dot->set(mfv.H->C(), w); // *mfv.C_dot = dynamics::_ddt_CIB(mfv.H->C(), w);
        mfv.q_dot->set(*mfv.q, w); // *mfv.q_dot = dynamics::_quat_kin_vel(*mfv.q, w);
        *mfv.w = w;
        mfv.eul_dot->set(w, *mfv.eul); // *mfv.eul_dot = dynamics::_wB_BI_to_eul_dot(w, *mfv.eul);
        *mfv.wq = wq;
    }
    void Frame::_set(const dynamics::LinearVelocity& v){
        MutableFrameView mfv = view();
        *mfv.v = v;
    }
    // void Frame::_set(const dynamics::LinearAcceleration& a){
    //     MutableFrameView mfv = view();
    //     *mfv.a = a;
    // }
    void Frame::_set(const dynamics::Gravity& g){
        MutableFrameView mfv = view();
        *mfv.g = g;
    }

    /** @deprecated */
    // void Frame::set(const dynamics::RigidBodyState& rigidBodyState){
    //     set(rigidBodyState.p);
    //     set(rigidBodyState.v);
    //     set(rigidBodyState.q);
    //     set(rigidBodyState.w);
    // }

    // void Frame::set(const geography::GeographicState& geographicState){
    //     set(ECEFFrame.pE_from_lat_lon_alt(geographicState));
    // }

    /** @deprecated */
    // void Frame::set(const SetOptions& opts){
    //     if (opts.H.has_value())         { _set(dynamics::HomogenousFrameTransformationMatrix{ *opts.H }); }
    //     if (opts.C.has_value())         { _set(dynamics::OrientationMatrix{ *opts.C }); }
    //     if (opts.p.has_value())         { _set(dynamics::Position{ *opts.p }); }
    //     if (opts.q.has_value())         { _set(dynamics::OrientationQuaternion{ *opts.q }); }
    //     if (opts.eul.has_value())       { _set(dynamics::EulerAngles{ *opts.eul }); }
    //     if (opts.C_dot.has_value())     { _set(dynamics::OrientationMatrixRate{ *opts.C_dot }); }
    //     if (opts.q_dot.has_value())     { _set(dynamics::OrientationQuaternionRate{ *opts.q_dot }); }
    //     if (opts.w.has_value())         { _set(dynamics::AngularVelocity{ *opts.w }); }
    //     if (opts.eul_dot.has_value())   { _set(dynamics::EulerAngleRates{ *opts.eul_dot }); }
    //     if (opts.wq.has_value())        { _set(dynamics::AngularVelocityQuaternion{ *opts.wq }); }
    //     if (opts.v.has_value())         { _set(dynamics::LinearVelocity{ *opts.v }); }
    //     // if (opts.a.has_value())         { _set(dynamics::LinearAcceleration{ *opts.a }); }
    //     if (opts.g.has_value())         { _set(dynamics::Gravity{ *opts.g }); }
    //     // opts.clear();
    // }

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
    // void SetOptions::clear() noexcept { *this = SetOptions{}; }


    Frame::Frame(std::string n, Frame* p) : name(n), parent(p) {};

    Frame::~Frame() {
        // DestruC_tor for frame A

        // Parent side cleanup (for each parent P in dependent_on)
        for (auto it = dependent_on.begin(); it != dependent_on.end(); ) {
            Frame* parent = *it;
            parent->dependents.erase(this); // Remove A from P’s dependents
            it = dependent_on.erase(it);    // Remove P from A’s dependent_on
        }
        // Child side (for each child C in dependents):
        for (auto it = dependents.begin(); it != dependents.end(); ) {
            Frame* dep = *it;
            dep->dependent_on.erase(this);  // Remove A from C’s dependent_on.
            it = dependents.erase(it);      // Remove C from A’s dependents
        }
    }


    void Frame::_add_as_direct_dependent(Frame* p) {
        dependents.insert(p);
        p->dependent_on.insert(this);
    }

    // GenericFrame::GenericFrame(std::string n, Frame* parent) : Frame(n), Parent(parent) {};
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

    Eigen::Vector3d rotate_vec(const Eigen::Vector3d& vA, const Frame& A, const Frame& B) {
        Eigen::Matrix3d CRA = CRF(A);
        Eigen::Matrix3d CRB = CRF(B);
        Eigen::Vector3d vB = CRB * CRA.transpose() * vA;
        return vB;
    }

    Eigen::Vector3d rotate_vec(const Eigen::Vector3d& vA, const Frame& A, const ECEFFrame&) {
        Eigen::Matrix3d CRA = CRF(A);
        Eigen::Vector3d vB = CRA.transpose() * vA;
        return vB;
    }

    Eigen::Vector3d rotate_vec(const Eigen::Vector3d& vA, const ECEFFrame&, const Frame& B) {
        Eigen::Matrix3d CRB = CRF(B);
        Eigen::Vector3d vB = CRB * vA;
        return vB;
    }

    /*
    Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const Frame& A, const Frame& B) {
        const Eigen::Matrix3d CRA = CRF(A);
        const Eigen::Matrix3d CRB = CRF(B);
        return CRB * (CRA.transpose() * pA + pRF(A) - pRF(B));
    }

    Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const Frame& A, const ECEFFrame&) {
        const Eigen::Matrix3d CRA = CRF(A);
        return CRA.transpose() * pA + pRF(A);
    }

    Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const ECEFFrame&, const Frame& B) {
        const Eigen::Matrix3d CRB = CRF(B);
        return CRB * (pA - pRF(B));
    }
    */

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

    // Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const Frame& A, const Frame& B) {
    //     return transforms::apply_H(HRF(B) * transforms::make_Hinv(HRF(A)), pA);
    // }

    // Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const Frame& A, const ECEFFrame&) {
    //     return transforms::apply_H(transforms::make_Hinv(HRF(A)), pA);
    // }

    // Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const ECEFFrame&, const Frame& B) {
    //     return transforms::apply_H(HRF(B), pA);
    // }




        // if (opts.p.has_value())         { set(*opts.p); }
        // if (opts.H.has_value())         { set(*opts.H); }
        // if (opts.C.has_value())         { set(*opts.C); }
        // if (opts.q.has_value())         { set(*opts.q); }
        // if (opts.eul.has_value())       { set(*opts.eul); }
        // if (opts.C_dot.has_value())     { set(*opts.C_dot); }
        // if (opts.q_dot.has_value())     { set(*opts.q_dot); }
        // if (opts.w.has_value())         { set(*opts.w); }
        // if (opts.eul_dot.has_value())   { set(*opts.eul_dot); }
        // if (opts.wq.has_value())        { set(*opts.wq); }
        // if (opts.v.has_value())         { set(*opts.v); }
        // if (opts.g.has_value())         { set(*opts.g); }
        // if (opts.alpha.has_value())     { set(*opts.alpha); }
        // if (opts.beta.has_value())      { set(*opts.beta); }
        // if (opts.lat.has_value() and opts.lon)  { set(*opts.lat); }




    /** @deprecated */
    // dynamics::Gravity ECEFFrame::gE(dynamics::Position pE){ return dynamics::Gravity{ -constants::g_earth * pE.normalized() }; }
  
}
