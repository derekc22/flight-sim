#pragma once
#include <Eigen/Dense>
#include <tuple>
#include <cmath>
#include <variant>
#include <optional>
#include <unordered_set>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"
#include "simulation/transforms/transforms.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include <algorithm>


namespace frames {

    struct Frame;
    struct NEDFrameECEF;
    struct FRDFrameECEF;
    struct FRDFrameNED;
    struct STABFrameFRD;
    struct WINDFrameSTAB;

    struct SetOptions {
        std::optional<dynamics::HomogenousFrameTransformationMatrix> H;
        std::optional<dynamics::OrientationMatrix> C;
        std::optional<dynamics::Position> p;
        std::optional<dynamics::OrientationQuaternion> q;
        std::optional<dynamics::EulerAngles> eul;
        std::optional<dynamics::OrientationMatrixRate> C_dot;
        std::optional<dynamics::OrientationQuaternionRate> q_dot;
        std::optional<dynamics::AngularVelocity> w;
        std::optional<dynamics::EulerAngleRates> eul_dot;
        std::optional<dynamics::AngularVelocityQuaternion> wq;
        std::optional<dynamics::LinearVelocity> v;
        std::optional<dynamics::Gravity> g;
        // void clear() noexcept;
    };

    struct MutableFrameView {
        dynamics::HomogenousFrameTransformationMatrix* H;
        dynamics::OrientationQuaternion* q;
        dynamics::EulerAngles* eul;
        dynamics::OrientationMatrixRate* C_dot;
        dynamics::OrientationQuaternionRate* q_dot;
        dynamics::AngularVelocity* w;
        dynamics::EulerAngleRates* eul_dot;
        dynamics::AngularVelocityQuaternion* wq;
        dynamics::LinearVelocity* v;
        dynamics::Gravity* g;
    };

    struct FrameView {
        const dynamics::HomogenousFrameTransformationMatrix* H;
        const dynamics::OrientationQuaternion* q;
        const dynamics::EulerAngles* eul;
        const dynamics::OrientationMatrixRate* C_dot;
        const dynamics::OrientationQuaternionRate* q_dot;
        const dynamics::AngularVelocity* w;
        const dynamics::EulerAngleRates* eul_dot;
        const dynamics::AngularVelocityQuaternion* wq;
        const dynamics::LinearVelocity* v;
        const dynamics::Gravity* g;
    };

    struct Frame {
        std::string name;
        Frame* parent = nullptr;

        Frame(std::string n, Frame* p);

        void _set(const dynamics::HomogenousFrameTransformationMatrix& H);
        void _set(const dynamics::OrientationMatrix& C);
        void _set(const dynamics::Position& p);
        void _set(const dynamics::OrientationQuaternion& q);
        void _set(const dynamics::EulerAngles& eul);
        void _set(const dynamics::OrientationMatrixRate& C_dot);
        void _set(const dynamics::OrientationQuaternionRate& q_dot);
        void _set(const dynamics::AngularVelocity& w);
        void _set(const dynamics::EulerAngleRates& eul_dot);
        void _set(const dynamics::AngularVelocityQuaternion& wq);
        void _set(const dynamics::LinearVelocity& v);
        void _set(const dynamics::Gravity& g);

        virtual MutableFrameView view() = 0;
        virtual FrameView view() const = 0;
        virtual ~Frame();

        void set(const SetOptions& opts);
        
        std::unordered_set<Frame*> dependents;
        std::unordered_set<Frame*> dependent_on;
        void _add_as_direct_dependent(Frame* p);
    };


    struct ECEFFrame { 
        std::string name = "ECEFFrame"; ;
    }; 

    inline const ECEFFrame ECEF {};

    // {ECEF} -> {NED}
    struct NEDFrameECEF : Frame {
        NEDFrameECEF();
        dynamics::HomogenousFrameTransformationMatrix HEN; 
        dynamics::OrientationQuaternion qEN;
        dynamics::EulerAngles eulEN;
        dynamics::OrientationMatrixRate CEN_dot;
        dynamics::OrientationQuaternionRate qEN_dot;
        dynamics::AngularVelocity wN_NE;
        dynamics::EulerAngleRates eulEN_dot;
        dynamics::AngularVelocityQuaternion wq_NE;
        dynamics::LinearVelocity vN_NE;
        dynamics::Gravity gN;

        MutableFrameView view() override;
        FrameView view() const override;
    };

    // {ECEF} -> {BODY}
    struct FRDFrameECEF : Frame {
        FRDFrameECEF();
        dynamics::HomogenousFrameTransformationMatrix HEB; 
        dynamics::OrientationQuaternion qEB;
        dynamics::EulerAngles eulEB;
        dynamics::OrientationMatrixRate CEB_dot;
        dynamics::OrientationQuaternionRate qEB_dot;
        dynamics::AngularVelocity wB_BE;
        dynamics::EulerAngleRates eulEB_dot;
        dynamics::AngularVelocityQuaternion wq_BE;
        dynamics::LinearVelocity vB_BE;
        dynamics::Gravity gB;

        MutableFrameView view() override;  
        FrameView view() const override;  
    };

    // {NED} -> {BODY}
    struct FRDFrameNED : Frame { 
        FRDFrameNED(NEDFrameECEF* pNEDFrameECEF);
        dynamics::HomogenousFrameTransformationMatrix HNB; 
        dynamics::OrientationQuaternion qNB;
        dynamics::EulerAngles eulNB;
        dynamics::OrientationMatrixRate CNB_dot;
        dynamics::OrientationQuaternionRate qNB_dot;
        dynamics::AngularVelocity wB_BN;
        dynamics::EulerAngleRates eulNB_dot;
        dynamics::AngularVelocityQuaternion wq_BN;
        dynamics::LinearVelocity vB_BN;
        dynamics::Gravity gB;

        MutableFrameView view() override; 
        FrameView view() const override; 
    };

    // {BODY} -> {STAB}
    struct STABFrameFRD : Frame {
        STABFrameFRD(FRDFrameNED* pFRDFrameNED);
        dynamics::HomogenousFrameTransformationMatrix HBS; 
        dynamics::OrientationQuaternion qBS;
        dynamics::EulerAngles eulBS;
        dynamics::OrientationMatrixRate CBS_dot;
        dynamics::OrientationQuaternionRate qBS_dot;
        dynamics::AngularVelocity wS_SB;
        dynamics::EulerAngleRates eulBS_dot;
        dynamics::AngularVelocityQuaternion wq_SB;
        dynamics::LinearVelocity vS_SB;
        dynamics::Gravity gS;

        MutableFrameView view() override; 
        FrameView view() const override; 
    };

    // {STAB} -> {WIND}
    struct WINDFrameSTAB : Frame {
        WINDFrameSTAB(STABFrameFRD* pSTABFrameFRD);
        dynamics::HomogenousFrameTransformationMatrix HSW; 
        dynamics::OrientationQuaternion qSW;
        dynamics::EulerAngles eulSW;
        dynamics::OrientationMatrixRate CSW_dot;
        dynamics::OrientationQuaternionRate qSW_dot;
        dynamics::AngularVelocity wW_WS;
        dynamics::EulerAngleRates eulSW_dot;
        dynamics::AngularVelocityQuaternion wq_WS;
        dynamics::LinearVelocity vW_WS;
        dynamics::Gravity gW;

        MutableFrameView view() override; 
        FrameView view() const override; 
    };



    /** @brief Obtains the rotation matrix from the root (ECEFFrame) to the frame F */
    Eigen::Matrix3d CRF(const Frame& F);

    /** @brief Rotates vector vA from frame {A} to frame {B}, producing vB */
    Eigen::Vector3d rotate_vec(const Eigen::Vector3d& vA, const Frame& A, const Frame& B);

    Eigen::Vector3d rotate_vec(const Eigen::Vector3d& vA, const Frame& A, const ECEFFrame&);

    Eigen::Vector3d rotate_vec(const Eigen::Vector3d& vA, const ECEFFrame&, const Frame& B);

    /** @brief Transforms point/position vector pA from frame {A} to frame {B}, producing pB
    If you have a point pA in one frame, transform_point(pA, A, B) re-expresses that same point in frame B */
    Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const Frame& A, const Frame& B);

    Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const Frame& A, const ECEFFrame&);

    Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const ECEFFrame&, const Frame& B);

    Eigen::Matrix4d HRF(const Frame& F);
    Eigen::Vector3d pRF(const Frame& F);



}
