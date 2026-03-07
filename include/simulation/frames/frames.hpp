#pragma once
#include <Eigen/Dense>
#include <tuple>
#include <cmath>
#include <variant>
#include <optional>
#include <unordered_set>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/global/global.hpp"
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

    /** @deprecated */
    // struct SetOptions{
    //     std::optional<Eigen::Matrix4d> H;
    //     std::optional<Eigen::Matrix3d> C;
    //     std::optional<Eigen::Vector3d> p;
    //     std::optional<Eigen::Quaterniond> q;
    //     std::optional<Eigen::Vector3d> eul;
    //     std::optional<Eigen::Matrix3d> C_dot;
    //     std::optional<Eigen::Quaterniond> q_dot;
    //     std::optional<Eigen::Vector3d> w;
    //     std::optional<Eigen::Vector3d> eul_dot;
    //     std::optional<Eigen::Quaterniond> wq;
    //     std::optional<Eigen::Vector3d> v;
    //     // std::optional<Eigen::Vector3d> a;
    //     std::optional<Eigen::Vector3d> g;

    //     // void clear() noexcept;
    // };

    /** @deprecated */
    // struct SetOptions{
    //     std::optional<dynamics::HomogenousFrameTransformationMatrix> H;
    //     std::optional<dynamics::OrientationMatrix> C;
    //     std::optional<dynamics::Position> p;
    //     std::optional<dynamics::OrientationQuaternion> q;
    //     std::optional<dynamics::EulerAngles> eul;
    //     std::optional<dynamics::OrientationMatrixRate> C_dot;
    //     std::optional<dynamics::OrientationQuaternionRate> q_dot;
    //     std::optional<dynamics::AngularVelocity> w;
    //     std::optional<dynamics::EulerAngleRates> eul_dot;
    //     std::optional<dynamics::AngularVelocityQuaternion> wq;
    //     std::optional<dynamics::LinearVelocity> v;
    //     std::optional<dynamics::Gravity> g;

    //     // void clear() noexcept;
    // };

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

        void set(const dynamics::HomogenousFrameTransformationMatrix& H);
        void set(const dynamics::OrientationMatrix& C);
        void set(const dynamics::Position& p);
        void set(const dynamics::OrientationQuaternion& q);
        void set(const dynamics::EulerAngles& eul);
        void set(const dynamics::OrientationMatrixRate& C_dot);
        void set(const dynamics::OrientationQuaternionRate& q_dot);
        void set(const dynamics::AngularVelocity& w);
        void set(const dynamics::EulerAngleRates& eul_dot);
        void set(const dynamics::AngularVelocityQuaternion& wq);
        void set(const dynamics::LinearVelocity& v);
        void set(const dynamics::Gravity& g);

        virtual MutableFrameView view() = 0;
        virtual FrameView view() const = 0;
        virtual ~Frame();
        // virtual void step(vehicles::StepOptions) = 0;
        // virtual void step(vehicles::StepOptionsStrict) = 0;

        /** @deprecated */
        // void set(const SetOptions& opts);
        void set(const SetOptions& opts);

        /** @deprecated */
        // void set(const dynamics::RigidBodyState& rigidBodyState);
        // dynamics::RigidBodyState RigidBodyState();

        /** @brief Obtains the rotation matrix from the root (ECEFFrame) to the frame F */
        dynamics::OrientationMatrix CRF(const Frame& F);
        
        /** @brief Rotates vector vA from frame {A} to frame {B}, producing vB */
        Eigen::Vector3d rotate_vec(const Eigen::Vector3d& vA, const Frame& A, const Frame& B);

        std::unordered_set<Frame*> dependents;
        std::unordered_set<Frame*> dependent_on;
        void add_as_direct_dependent(Frame* p);
    };

    /** @deprecated */
    // {PARENT, Y} -> {SELF, X}
    // struct GenericFrame : Frame {
    //     GenericFrame(std::string n, Frame* parent);
    //     dynamics::HomogenousFrameTransformationMatrix HYX; 
    //     dynamics::OrientationQuaternion qYX;
    //     dynamics::EulerAngles eulYX;
    //     dynamics::OrientationMatrixRate CYX_dot;
    //     dynamics::OrientationQuaternionRate qYX_dot;
    //     dynamics::AngularVelocity wX_XY;
    //     dynamics::AngularVelocityQuaternion wq_XY;
    //     dynamics::EulerAngleRates eulYX_dot;
    //     dynamics::LinearVelocity vX_XY;
    //     dynamics::Gravity gX;

    //     FrameView view() override;
    //     // FrameView view() const override;
    //     void step(const vehicles::_StepOptions& opts) override;
    // };

    struct ECEFFrame { 
        std::string name = "ECEFFrame"; 

        /** @deprecated */
        // dynamics::Gravity gE(dynamics::Position pE);
        // geography::GeographicState lat_lon_alt_from_pE(const dynamics::Position & pE);
        // dynamics::Position pE_from_lat_lon_alt(const geography::GeographicState& geographicState);
    }; 

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
        // void step(const vehicles::_StepOptions& opts) override;

        /** @deprecated */
        // dynamics::OrientationMatrix CEN_from_lat_lon(const geography::Latitude& lat, const geography::Longitude& lon);
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
        // void step(const vehicles::_StepOptions& opts) override;
    };

    // {NED} -> {BODY}
    struct FRDFrameNED : Frame { 
        // void Frame::set(const dynamics::RigidBodyState& rigidBodyState)

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
        // void step(const vehicles::_StepOptions& opts) override;
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
        // void step(const vehicles::_StepOptions& opts) override;

        /** @deprecated */
        // dynamics::OrientationMatrix CBS(const aerodynamics::AngleOfAttack& alpha) const;
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
        // void step(const vehicles::_StepOptions& opts) override;

        /** @deprecated */
        // dynamics::OrientationMatrix CSW(const aerodynamics::SideslipAngle& beta) const;
    };


}
