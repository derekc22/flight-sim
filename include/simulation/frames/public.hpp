#pragma once
#include <Eigen/Dense>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include "simulation/dynamics/public.hpp"

namespace frames {

    struct FrameView {
        const dynamics::HomogeneousTransformationMatrix* H;
        const dynamics::OrientationQuaternion* q;
        const dynamics::EulerAngles* eul;
        const dynamics::OrientationMatrixRate* C_dot;
        const dynamics::OrientationQuaternionRate* q_dot;
        const dynamics::AngularVelocity* w;
        const dynamics::EulerAngleRates* eul_dot;
        const dynamics::AngularVelocityQuaternion* wq;
        const dynamics::TranslationalVelocity* v;
        const dynamics::Gravity* g;
    };

    struct MutableFrameView {
        dynamics::HomogeneousTransformationMatrix* H;
        dynamics::OrientationQuaternion* q;
        dynamics::EulerAngles* eul;
        dynamics::OrientationMatrixRate* C_dot;
        dynamics::OrientationQuaternionRate* q_dot;
        dynamics::AngularVelocity* w;
        dynamics::EulerAngleRates* eul_dot;
        dynamics::AngularVelocityQuaternion* wq;
        dynamics::TranslationalVelocity* v;
        dynamics::Gravity* g;
    };

    struct SetOptions {
        std::optional<dynamics::HomogeneousTransformationMatrix> H;
        std::optional<dynamics::OrientationMatrix> C;
        std::optional<dynamics::Position> p;
        std::optional<dynamics::OrientationQuaternion> q;
        std::optional<dynamics::EulerAngles> eul;
        std::optional<dynamics::OrientationMatrixRate> C_dot;
        std::optional<dynamics::OrientationQuaternionRate> q_dot;
        std::optional<dynamics::AngularVelocity> w;
        std::optional<dynamics::EulerAngleRates> eul_dot;
        std::optional<dynamics::AngularVelocityQuaternion> wq;
        std::optional<dynamics::TranslationalVelocity> v;
        std::optional<dynamics::Gravity> g;
        // void clear() noexcept;
    };

    struct Frame {
        std::string name;
        Frame* parent = nullptr;

        Frame(std::string n, Frame* p);

        void _set(const dynamics::HomogeneousTransformationMatrix& H);
        void _set(const dynamics::OrientationMatrix& C);
        void _set(const dynamics::Position& p);
        void _set(const dynamics::OrientationQuaternion& q);
        void _set(const dynamics::EulerAngles& eul);
        void _set(const dynamics::OrientationMatrixRate& C_dot);
        void _set(const dynamics::OrientationQuaternionRate& q_dot);
        void _set(const dynamics::AngularVelocity& w);
        void _set(const dynamics::EulerAngleRates& eul_dot);
        void _set(const dynamics::AngularVelocityQuaternion& wq);
        void _set(const dynamics::TranslationalVelocity& v);
        void _set(const dynamics::Gravity& g);

        virtual MutableFrameView view() = 0;
        virtual FrameView view() const = 0;
        virtual ~Frame();

        void set(const SetOptions& opts);
        
        std::unordered_set<Frame*> dependents;
        std::unordered_set<Frame*> dependent_on;
        void add_as_direct_dependent(Frame* p);
    };

    struct ECEFFrame { 
        std::string name = "ECEFFrame"; ;
    }; 

    // {ECEF} -> {NED}
    struct NEDFrameECEF : Frame {
        NEDFrameECEF();
        dynamics::HomogeneousTransformationMatrix HEN; 
        dynamics::OrientationQuaternion qEN;
        dynamics::EulerAngles eulEN;
        dynamics::OrientationMatrixRate CEN_dot;
        dynamics::OrientationQuaternionRate qEN_dot;
        dynamics::AngularVelocity wN_NE;
        dynamics::EulerAngleRates eulEN_dot;
        dynamics::AngularVelocityQuaternion wq_NE;
        dynamics::TranslationalVelocity vN_NE;
        dynamics::Gravity gN;

        MutableFrameView view() override;
        FrameView view() const override;
    };

    // {ECEF} -> {BODY}
    struct FRDFrameECEF : Frame {
        FRDFrameECEF();
        dynamics::HomogeneousTransformationMatrix HEB; 
        dynamics::OrientationQuaternion qEB;
        dynamics::EulerAngles eulEB;
        dynamics::OrientationMatrixRate CEB_dot;
        dynamics::OrientationQuaternionRate qEB_dot;
        dynamics::AngularVelocity wB_BE;
        dynamics::EulerAngleRates eulEB_dot;
        dynamics::AngularVelocityQuaternion wq_BE;
        dynamics::TranslationalVelocity vB_BE;
        dynamics::Gravity gB;

        MutableFrameView view() override;  
        FrameView view() const override;  
    };

    // {NED} -> {BODY}
    struct FRDFrameNED : Frame { 
        FRDFrameNED(NEDFrameECEF* pNEDFrameECEF);
        dynamics::HomogeneousTransformationMatrix HNB; 
        dynamics::OrientationQuaternion qNB;
        dynamics::EulerAngles eulNB;
        dynamics::OrientationMatrixRate CNB_dot;
        dynamics::OrientationQuaternionRate qNB_dot;
        dynamics::AngularVelocity wB_BN;
        dynamics::EulerAngleRates eulNB_dot;
        dynamics::AngularVelocityQuaternion wq_BN;
        dynamics::TranslationalVelocity vB_BN;
        dynamics::Gravity gB;

        MutableFrameView view() override; 
        FrameView view() const override; 
    };

    // {BODY} -> {STAB}
    struct STABFrameFRD : Frame {
        STABFrameFRD(FRDFrameNED* pFRDFrameNED);
        dynamics::HomogeneousTransformationMatrix HBS; 
        dynamics::OrientationQuaternion qBS;
        dynamics::EulerAngles eulBS;
        dynamics::OrientationMatrixRate CBS_dot;
        dynamics::OrientationQuaternionRate qBS_dot;
        dynamics::AngularVelocity wS_SB;
        dynamics::EulerAngleRates eulBS_dot;
        dynamics::AngularVelocityQuaternion wq_SB;
        dynamics::TranslationalVelocity vS_SB;
        dynamics::Gravity gS;

        MutableFrameView view() override; 
        FrameView view() const override; 
    };

    // {STAB} -> {WIND}
    struct WINDFrameSTAB : Frame {
        WINDFrameSTAB(STABFrameFRD* pSTABFrameFRD);
        dynamics::HomogeneousTransformationMatrix HSW; 
        dynamics::OrientationQuaternion qSW;
        dynamics::EulerAngles eulSW;
        dynamics::OrientationMatrixRate CSW_dot;
        dynamics::OrientationQuaternionRate qSW_dot;
        dynamics::AngularVelocity wW_WS;
        dynamics::EulerAngleRates eulSW_dot;
        dynamics::AngularVelocityQuaternion wq_WS;
        dynamics::TranslationalVelocity vW_WS;
        dynamics::Gravity gW;

        MutableFrameView view() override; 
        FrameView view() const override; 
    };

    inline const ECEFFrame ECEF {};

    /** @brief Performs a coordinate transformation on a vector vA from frame {A} to frame {B}, producing vB */
    Eigen::Vector3d transform_vec(const Eigen::Vector3d& vA, const Frame& A, const Frame& B);
    Eigen::Vector3d transform_vec(const Eigen::Vector3d& vA, const Frame& A, const ECEFFrame&);
    Eigen::Vector3d transform_vec(const Eigen::Vector3d& vA, const ECEFFrame&, const Frame& B);

    /** @brief Performs a homogeneous transformation (translation + coordinate transformation) on a point/position vector pA from frame {A} to frame {B}, producing pB
    If you have point expressed in frame A and described by pA, transform_point(pA, A, B) re-expresses that same point in frame B */
    Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const Frame& A, const Frame& B);
    Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const Frame& A, const ECEFFrame&);
    Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const ECEFFrame&, const Frame& B);

}
