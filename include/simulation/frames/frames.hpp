#pragma once
#include <Eigen/Dense>
#include <tuple>
#include <cmath>
#include <variant>
#include <optional>
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/global/global.hpp"
#include "simulation/transforms/transforms.hpp"

namespace frames {

    struct StepOptions{
        std::optional<Eigen::Matrix4d> H;
        std::optional<Eigen::Matrix3d> C;
        std::optional<Eigen::Vector3d> p;
        std::optional<Eigen::Quaterniond> q;
        std::optional<Eigen::Vector3d> eul;
        std::optional<Eigen::Matrix3d> C_dot;
        std::optional<Eigen::Quaterniond> q_dot;
        std::optional<Eigen::Vector3d> w;
        std::optional<Eigen::Vector3d> eul_dot;
        std::optional<Eigen::Quaterniond> wq;
        std::optional<Eigen::Vector3d> v;
        // std::optional<Eigen::Vector3d> a;
        std::optional<Eigen::Vector3d> g;

        void clear() noexcept;
    };

    struct StepOptionsStrict{
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
        // std::optional<dynamics::LinearAcceleration> a;
        std::optional<dynamics::Gravity> g;

        void clear() noexcept;
    };

    struct FrameView {
        dynamics::HomogenousFrameTransformationMatrix* H;
        dynamics::OrientationQuaternion* q;
        dynamics::EulerAngles* eul;
        dynamics::OrientationMatrixRate* C_dot;
        dynamics::OrientationQuaternionRate* q_dot;
        dynamics::AngularVelocity* w;
        dynamics::EulerAngleRates* eul_dot;
        dynamics::AngularVelocityQuaternion* wq;
        dynamics::LinearVelocity* v;
        // dynamics::LinearAcceleration* a;
        dynamics::Gravity* g;
    };

    struct Frame {
        std::string name;
        explicit Frame(std::string n) : name(n) {};

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
        // void set(const dynamics::LinearAcceleration& a);
        void set(const dynamics::Gravity& g);

        virtual FrameView view() = 0;
        // virtual FrameView view() const = 0;
        virtual ~Frame() = default;

        void step(const StepOptionsStrict& opts);
        void step(const StepOptions& opts);
        void step(const dynamics::RigidBodyState& rbs);


        dynamics::RigidBodyState RigidBodyState();

    };

    // {SELF, X} -> {PARENT, Y}
    struct GenericFrame : Frame {
        Frame& Parent;
        GenericFrame(std::string n, Frame& parent) : Frame(n), Parent(parent) {};
        dynamics::HomogenousFrameTransformationMatrix HYX; 
        dynamics::OrientationQuaternion qYX;
        dynamics::EulerAngles eulYX; // ZYX
        dynamics::OrientationMatrixRate CYX_dot;
        dynamics::OrientationQuaternionRate qYX_dot;
        dynamics::AngularVelocity wX_XY;
        dynamics::AngularVelocityQuaternion wq_XY;
        dynamics::EulerAngleRates eulYX_dot;
        dynamics::LinearVelocity vX_XY;
        // dynamics::LinearAcceleration aX_XY;
        dynamics::Gravity gX;

        FrameView view() override;
        // FrameView view() const override;
    };

    struct ECEFFrame { std::string name = "ECEFFrame"; }; 

    // {ECEF} -> {NED}
    struct NEDFrameECEF : Frame {
        NEDFrameECEF() : Frame("NEDFrameECEF") {};
        dynamics::HomogenousFrameTransformationMatrix HEN; 
        dynamics::OrientationQuaternion qEN;
        dynamics::EulerAngles eulEN; // ZYX
        dynamics::OrientationMatrixRate CEN_dot;
        dynamics::OrientationQuaternionRate qEN_dot;
        dynamics::AngularVelocity wN_NE;
        dynamics::EulerAngleRates eulEN_dot;
        dynamics::AngularVelocityQuaternion wq_NE;
        dynamics::LinearVelocity vN_NE;
        // dynamics::LinearAcceleration aN_NE;
        dynamics::Gravity gN;

        FrameView view() override;
        // FrameView view() const override;
    };

    /// {ECEF} -> {BODY}
    struct FRDFrameECEF : Frame {
        FRDFrameECEF() : Frame("FRDFrameECEF") {};
        dynamics::HomogenousFrameTransformationMatrix HEB; 
        dynamics::OrientationQuaternion qEB;
        dynamics::EulerAngles eulEB; // ZYX
        dynamics::OrientationMatrixRate CEB_dot;
        dynamics::OrientationQuaternionRate qEB_dot;
        dynamics::AngularVelocity wB_BE;
        dynamics::EulerAngleRates eulEB_dot;
        dynamics::AngularVelocityQuaternion wq_BE;
        dynamics::LinearVelocity vB_BE;
        // dynamics::LinearAcceleration aB_BE;
        dynamics::Gravity gB;

        FrameView view() override;  
        // FrameView view() const override;  
    };

    /// {NED} -> {BODY}
    struct FRDFrameNED : Frame {
        FRDFrameNED() : Frame("FRDFrameNED") {};
        dynamics::HomogenousFrameTransformationMatrix HNB; 
        dynamics::OrientationQuaternion qNB;
        dynamics::EulerAngles eulNB; // ZYX
        dynamics::OrientationMatrixRate CNB_dot;
        dynamics::OrientationQuaternionRate qNB_dot;
        dynamics::AngularVelocity wB_BN;
        dynamics::EulerAngleRates eulNB_dot;
        dynamics::AngularVelocityQuaternion wq_BN;
        dynamics::LinearVelocity vB_BN;
        // dynamics::LinearAcceleration aB_BN;
        dynamics::Gravity gB;

        FrameView view() override; 
        // FrameView view() const override; 
    };

    Eigen::Matrix3d CEN_from_lat_lon(double lat, double lon);

    std::array<double, 3> lat_lon_alt_from_xECEF(const Eigen::Vector3d& xECEF);






    namespace common {
        Eigen::Vector3d gECEF(Eigen::Vector3d pE_BE);
    }


}
