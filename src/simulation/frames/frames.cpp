#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include "simulation/frames/frames.hpp"
#include "simulation/transforms/transforms.hpp"
#include "simulation/global/global.hpp"

namespace frames {

    Eigen::Matrix3d CEN_from_lat_lon(double lat, double lon) {
        Eigen::Matrix3d CEN;
        CEN     <<   -std::sin(lat) * std::cos(lon),  -std::sin(lat) * std::sin(lon),   std::cos(lat),
                                     -std::sin(lon),                   std::cos(lon),               0,
                     -std::cos(lat) * std::cos(lon),  -std::cos(lat) * std::sin(lon),  -std::sin(lat);
        return CEN;
    };

    std::array<double, 3> lat_lon_alt_from_xECEF(const Eigen::Vector3d& xECEF) {
        double x = xECEF(0);
        double y = xECEF(1);
        double z = xECEF(2);

        double p = std::sqrt(x * x + y * y);
        double r = std::sqrt(x * x + y * y + z * z);

        // Longitude (radians)
        double lon = std::atan2(y, x);

        // Geocentric latitude (radians)
        double lat = std::atan2(z, p);

        // Altitude above the spherical Earth (meters)
        double alt = r - global::r_earth;

        return { lat, lon, alt };
    }




    dynamics::RigidBodyState Frame::RigidBodyState(){
        FrameView fv = view();
        return dynamics::RigidBodyState { 
            .pI_BI = dynamics::Position { fv.H->p() }, 
            .vB_BI = *fv.v, .qIB = *fv.q, .wB_BI = *fv.w
        };
    }


    FrameView GenericFrame::view() {
        // return { &HYX, &qYX, &eulYX, &CYX_dot, &qYX_dot, &wX_XY, &eulYX_dot, &wq_XY, &vX_XY, &aX_XY, &gX };
        return { &HYX, &qYX, &eulYX, &CYX_dot, &qYX_dot, &wX_XY, &eulYX_dot, &wq_XY, &vX_XY, &gX };
    }
    // FrameView GenericFrame::view() const {
    //     return const_cast<GenericFrame*>(this)->view();
    // }

    FrameView NEDFrameECEF::view() {
        // return { &HEN, &qEN, &eulEN, &CEN_dot, &qEN_dot, &wN_NE, &eulEN_dot, &wq_NE, &vN_NE, &aN_NE, &gN };
        return { &HEN, &qEN, &eulEN, &CEN_dot, &qEN_dot, &wN_NE, &eulEN_dot, &wq_NE, &vN_NE, &gN };
    }
    // FrameView NEDFrameECEF::view() const {
    //     return const_cast<NEDFrameECEF*>(this)->view();
    // }

    FrameView FRDFrameECEF::view() {
        // return { &HEB, &qEB, &eulEB, &CEB_dot, &qEB_dot, &wB_BE, &eulEB_dot, &wq_BE, &vB_BE, &aB_BE, &gB };
        return { &HEB, &qEB, &eulEB, &CEB_dot, &qEB_dot, &wB_BE, &eulEB_dot, &wq_BE, &vB_BE, &gB };
    }
    // FrameView FRDFrameECEF::view() const {
    //     return const_cast<FRDFrameECEF*>(this)->view();
    // }

    FrameView FRDFrameNED::view() {
        // return { &HNB, &qNB, &eulNB, &CNB_dot, &qNB_dot, &wB_BN, &eulNB_dot, &wq_BN, &vB_BN, &aB_BN, &gB };
        return { &HNB, &qNB, &eulNB, &CNB_dot, &qNB_dot, &wB_BN, &eulNB_dot, &wq_BN, &vB_BN, &gB };
    }
    // FrameView FRDFrameNED::view() const {
    //     return const_cast<FRDFrameNED*>(this)->view();
    // }









    void Frame::set(const dynamics::HomogenousFrameTransformationMatrix& H){
        set(H.C());
        set(H.p());
    }
    void Frame::set(const dynamics::OrientationMatrix& C){
        FrameView fv = view();
        fv.H->set(C);
        fv.q->set(C); // *fv.q = dynamics::OrientationQuaternion{ transforms::rot2quat(C.data) };
        fv.eul->set(C, "ZYX"); // *fv.eul = dynamics::EulerAngles{ transforms::C2eul_intr(C.data, "ZYX") };
    }
    void Frame::set(const dynamics::Position& p){
        FrameView fv = view();
        fv.H->set(p);
    }
    void Frame::set(const dynamics::OrientationQuaternion& q){
        FrameView fv = view();
        *fv.q = q;
        fv.H->set(q); // fv.H->set(transforms::quat2rot(q.data));
        fv.eul->set(q, "ZYX"); // *fv.eul = dynamics::EulerAngles{ transforms::quatC2eul_intr(q.data, "ZYX") };
    }
    void Frame::set(const dynamics::EulerAngles& eul){
        FrameView fv = view();
        fv.H->set(eul, "ZYX"); // fv.H->set(transforms::eul2C_intr(eul.psi(), eul.theta(), eul.phi(), "ZYX"));
        fv.q->set(eul, "ZYX"); // *fv.q = dynamics::OrientationQuaternion{ transforms::eul2quatC_intr(eul.psi(), eul.theta(), eul.phi(), "ZYX") };
        *fv.eul = eul;
    }
    void Frame::set(const dynamics::OrientationMatrixRate& C_dot){
        FrameView fv = view();
        *fv.C_dot = C_dot;
        dynamics::AngularVelocity w = dynamics::CIB_dot2wB_BI(*fv.C_dot, fv.H->C());
        fv.q_dot->set(*fv.q, w); // *fv.q_dot = dynamics::quat_kin_vel(*fv.q, w);
        // fv.q_dot->set(C_dot, fv.H->C(), *fv.q); // *fv.q_dot = dynamics::CIB_dot2qIB_dot(C_dot, fv.H->C(), *fv.q);
        *fv.w = w;
        fv.eul_dot->set(w, *fv.eul); // *fv.eul_dot = dynamics::wB_BI2eul_dot(w, *fv.eul);
        fv.wq->set(w);
    }
    void Frame::set(const dynamics::OrientationQuaternionRate& q_dot){
        FrameView fv = view();
        fv.C_dot->set(q_dot, *fv.q, fv.H->C()); // *fv.C_dot = dynamics::qIB_dot2CIB_dot(q_dot, *fv.q, fv.H->C());
        *fv.q_dot = q_dot;
        dynamics::AngularVelocity w = dynamics::qIB_dot2wB_BI(q_dot, *fv.q);
        *fv.w = w;
        fv.eul_dot->set(w, *fv.eul); // *fv.eul_dot = dynamics::wB_BI2eul_dot(w, *fv.eul);
        fv.wq->set(w);
    }
    void Frame::set(const dynamics::AngularVelocity& w){
        FrameView fv = view();
        fv.C_dot->set(fv.H->C(), w); // *fv.C_dot = dynamics::ddt_CIB(fv.H->C(), w);
        fv.q_dot->set(*fv.q, w); // *fv.q_dot = dynamics::quat_kin_vel(*fv.q, w);
        *fv.w = w;
        fv.eul_dot->set(w, *fv.eul); // *fv.eul_dot = dynamics::wB_BI2eul_dot(w, *fv.eul);
        fv.wq->set(w);
    }
    void Frame::set(const dynamics::EulerAngleRates& eul_dot){
        FrameView fv = view();
        dynamics::AngularVelocity w = dynamics::eul_dot2wB_BI(eul_dot, *fv.eul);
        fv.C_dot->set(fv.H->C(), w); // *fv.C_dot = dynamics::ddt_CIB(fv.H->C(), w);
        fv.q_dot->set(*fv.q, w); // *fv.q_dot = dynamics::quat_kin_vel(*fv.q, w);
        *fv.w = w;
        *fv.eul_dot = eul_dot;
        fv.wq->set(w);
    }
    void Frame::set(const dynamics::AngularVelocityQuaternion& wq){
        FrameView fv = view();
        dynamics::AngularVelocity w = wq.wB_BI();
        fv.C_dot->set(fv.H->C(), w); // *fv.C_dot = dynamics::ddt_CIB(fv.H->C(), w);
        fv.q_dot->set(*fv.q, w); // *fv.q_dot = dynamics::quat_kin_vel(*fv.q, w);
        *fv.w = w;
        fv.eul_dot->set(w, *fv.eul); // *fv.eul_dot = dynamics::wB_BI2eul_dot(w, *fv.eul);
        *fv.wq = wq;
    }
    void Frame::set(const dynamics::LinearVelocity& v){
        FrameView fv = view();
        *fv.v = v;
    }
    // void Frame::set(const dynamics::LinearAcceleration& a){
    //     FrameView fv = view();
    //     *fv.a = a;
    // }
    void Frame::set(const dynamics::Gravity& g){
        FrameView fv = view();
        *fv.g = g;
    }


    void Frame::step(const dynamics::RigidBodyState& rbs){
        set(rbs.pI_BI);
        set(rbs.vB_BI);
        set(rbs.qIB);
        set(rbs.wB_BI);
    }

    void Frame::step(const StepOptionsStrict& opts){
        if (opts.H)         { set(*opts.H); }
        if (opts.C)         { set(*opts.C); }
        if (opts.p)         { set(*opts.p); }
        if (opts.q)         { set(*opts.q); }
        if (opts.eul)       { set(*opts.eul); }
        if (opts.C_dot)     { set(*opts.C_dot); }
        if (opts.q_dot)     { set(*opts.q_dot); }
        if (opts.w)         { set(*opts.w); }
        if (opts.eul_dot)   { set(*opts.eul_dot); }
        if (opts.wq)        { set(*opts.wq); }
        if (opts.v)         { set(*opts.v); }
        // if (opts.a)         { set(*opts.a); }
        if (opts.g)         { set(*opts.g); }
    }

    void Frame::step(const StepOptions& opts){
        if (opts.H)         { set(dynamics::HomogenousFrameTransformationMatrix{ *opts.H }); }
        if (opts.C)         { set(dynamics::OrientationMatrix{ *opts.C }); }
        if (opts.p)         { set(dynamics::Position{ *opts.p }); }
        if (opts.q)         { set(dynamics::OrientationQuaternion{ *opts.q }); }
        if (opts.eul)       { set(dynamics::EulerAngles{ *opts.eul }); }
        if (opts.C_dot)     { set(dynamics::OrientationMatrixRate{ *opts.C_dot }); }
        if (opts.q_dot)     { set(dynamics::OrientationQuaternionRate{ *opts.q_dot }); }
        if (opts.w)         { set(dynamics::AngularVelocity{ *opts.w }); }
        if (opts.eul_dot)   { set(dynamics::EulerAngleRates{ *opts.eul_dot }); }
        if (opts.wq)        { set(dynamics::AngularVelocityQuaternion{ *opts.wq }); }
        if (opts.v)         { set(dynamics::LinearVelocity{ *opts.v }); }
        // if (opts.a)         { set(dynamics::LinearAcceleration{ *opts.a }); }
        if (opts.g)         { set(dynamics::Gravity{ *opts.g }); }
    }


    void StepOptionsStrict::clear() noexcept { *this = StepOptionsStrict{}; }
    void StepOptions::clear() noexcept { *this = StepOptions{}; }


    namespace common {
        // Get local ECEF gravity vector expressed WRT the ECEF frame based on your current position WRT the ECEF frame expressed WRT the ECEF frame
        Eigen::Vector3d gECEF(Eigen::Vector3d pE_BE){ return -global::gravity * pE_BE.normalized(); }
    }

}
