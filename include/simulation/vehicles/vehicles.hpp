#pragma once
#include <Eigen/Dense>
#include "simulation/frames/frames.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/control/control.hpp"
#include "simulation/geography/geography.hpp"
#include "simulation/atmospheric/atmospheric.hpp"



namespace vehicles {

    struct NEDFrameECEFStepOptions {

        /** @note NEDFrameECEF, {ECEF} -> {NED}
            Assumed inertial. Once initialized, NEDFrameECEF remains fixed
            Position: Determined by input
            Orientation: Uniquely determined by initial Position
            Orientation rate: None
            Linear velocity: None
            Gravity: Uniquely determined by initial Position and initial Orientation
        */
        // std::optional<dynamics::HomogenousFrameTransformationMatrix> HEN;
        // std::optional<dynamics::OrientationMatrix> CEN;
        // std::optional<dynamics::Position> pE_NE;
        // std::optional<dynamics::OrientationQuaternion> qEN;
        // std::optional<dynamics::EulerAngles> eulEN;
        // std::optional<dynamics::OrientationMatrixRate> CEN_dot;
        // std::optional<dynamics::OrientationQuaternionRate> qEN_dot;
        // std::optional<dynamics::AngularVelocity> wN_NE;
        // std::optional<dynamics::EulerAngleRates> eulEN_dot;
        // std::optional<dynamics::AngularVelocityQuaternion> wq_NE;
        // std::optional<dynamics::LinearVelocity> vN_NE;
        // std::optional<dynamics::Gravity> gN;
        std::optional<geography::Latitude> lat_NE;
        std::optional<geography::Longitude> lon_NE;
        std::optional<geography::Altitude> alt_NE;
        std::optional<geography::GeographicState> gps_NE; // container
    };

    struct FRDFrameECEFStepOptions {

        /** @note FRDFrameECEF, {ECEF} -> {FRD}
            Non-inertial
            Position: Determined by input OR (uniquely determined by NEDFrameECEF Orientation AND FRDFrameNED Position AND NEDFrameECEF Position)
            Orientation: Determined by input OR (uniquely determined by NEDFrameECEF Orientation AND FRDFrameNED Orientation)
            Orientation rate: Determined by input OR (equal to FRDFrameNED Angular velocity)
            Linear velocity: Determined by input OR (equal to FRDFrameNED Linear velocity)
            Gravity: Uniquely determined by Position and Orientation
        */
        std::optional<dynamics::HomogenousFrameTransformationMatrix> HEB; 
        std::optional<dynamics::OrientationMatrix> CEB;
        std::optional<dynamics::Position> pE_BE;
        std::optional<dynamics::OrientationQuaternion> qEB;
        std::optional<dynamics::EulerAngles> eulEB;
        std::optional<dynamics::OrientationMatrixRate> CEB_dot;
        std::optional<dynamics::OrientationQuaternionRate> qEB_dot;
        std::optional<dynamics::AngularVelocity> wB_BE;
        std::optional<dynamics::EulerAngleRates> eulEB_dot;
        std::optional<dynamics::AngularVelocityQuaternion> wq_BE;
        std::optional<dynamics::LinearVelocity> vB_BE;
        // std::optional<dynamics::Gravity> gB;
        std::optional<geography::Latitude> lat_BE;
        std::optional<geography::Longitude> lon_BE;
        std::optional<geography::Altitude> alt_BE;
        std::optional<dynamics::RigidBodyState> rbs_BE; // container
        std::optional<geography::GeographicState> gps_BE; // container
    };

    struct FRDFrameNEDStepOptions {

        /** @note FRDFrameNED, {NED} -> {FRD}
            Non-inertial
            Position: Determined by input OR (uniquely determined by NEDFrameECEF Orientation AND FRDFrameECEF Position AND NEDFrameECEF Position)
            Orientation: Determined by input OR (uniquely determined by NEDFrameECEF Orientation AND FRDFrameECEF Orientation)
            Orientation rate: Determined by input OR (equal to FRDFrameECEF Angular velocity)
            Linear velocity: Determined by input OR (equal to FRDFrameECEF Linear velocity)
            Gravity: Equal to FRDFrameECEF Gravity
        */
        std::optional<dynamics::HomogenousFrameTransformationMatrix> HNB; 
        std::optional<dynamics::OrientationMatrix> CNB;
        std::optional<dynamics::Position> pN_BN;
        std::optional<dynamics::OrientationQuaternion> qNB;
        std::optional<dynamics::EulerAngles> eulNB;
        std::optional<dynamics::OrientationMatrixRate> CNB_dot;
        std::optional<dynamics::OrientationQuaternionRate> qNB_dot;
        std::optional<dynamics::AngularVelocity> wB_BN;
        std::optional<dynamics::EulerAngleRates> eulNB_dot;
        std::optional<dynamics::AngularVelocityQuaternion> wq_BN;
        std::optional<dynamics::LinearVelocity> vB_BN;
        // std::optional<dynamics::Gravity> gB;
        std::optional<dynamics::RigidBodyState> rbs_BN; // container
    };

    struct STABFrameFRDStepOptions {

        /** @note STABFrameFRD, {FRD} -> {STAB}
            Non-inertial
            Position: 0
            Orientation: Uniquely determined by AngleOfAttack
            Orientation rate: Determined by input OR (assume 0 ???) OR (finite difference ???) OR (ignore entirely, for now, since they're not actually needed for anything)
            Linear velocity: 0
            Gravity: Uniquely determined by Orientation AND FRDFrameNED Gravity 
            AngleOfAttack: Determined by input
        */
        // std::optional<dynamics::HomogenousFrameTransformationMatrix> HBS; 
        // std::optional<dynamics::OrientationMatrix> CBS;
        // std::optional<dynamics::Position> pB_SB;
        // std::optional<dynamics::OrientationQuaternion> qBS;
        // std::optional<dynamics::EulerAngles> eulBS;
        // std::optional<dynamics::OrientationMatrixRate> CBS_dot;
        // std::optional<dynamics::OrientationQuaternionRate> qBS_dot;
        // std::optional<dynamics::AngularVelocity> wS_SB;
        // std::optional<dynamics::EulerAngleRates> eulBS_dot;
        // std::optional<dynamics::AngularVelocityQuaternion> wq_SB;
        // std::optional<dynamics::LinearVelocity> vS_SB;
        // std::optional<dynamics::Gravity> gS;
        std::optional<aerodynamics::AngleOfAttack> alpha;
        std::optional<aerodynamics::AerodynamicState> ads; // container
    };

    struct WINDFrameSTABStepOptions {

        /** @note WINDFrameSTAB, {STAB} -> {WIND}
            Non-inertial
            Position: 0
            Orientation: Uniquely determined by SideslipAngle
            Orientation rate: Determined by input OR (assume 0 ???) OR (finite difference ???) OR (ignore entirely, for now, since they're not actually needed for anything)
            Linear velocity: 0
            Gravity: Uniquely determined by Orientation AND STABFrameFRD Gravity 
            SideslipAngle: Determined by input
        */
        // std::optional<dynamics::HomogenousFrameTransformationMatrix> HSW; 
        // std::optional<dynamics::OrientationMatrix> CSW;
        // std::optional<dynamics::Position> pS_WS;
        // std::optional<dynamics::OrientationQuaternion> qSW;
        // std::optional<dynamics::EulerAngles> eulSW;
        // std::optional<dynamics::OrientationMatrixRate> CSW_dot;
        // std::optional<dynamics::OrientationQuaternionRate> qSW_dot;
        // std::optional<dynamics::AngularVelocity> wW_WS;
        // std::optional<dynamics::EulerAngleRates> eulSW_dot;
        // std::optional<dynamics::AngularVelocityQuaternion> wq_WS;
        // std::optional<dynamics::LinearVelocity> vW_WS;
        // std::optional<dynamics::Gravity> gW;
        std::optional<aerodynamics::SideslipAngle> beta;
        std::optional<aerodynamics::AerodynamicState> ads; // container
    };

    struct StepOptions {
        std::optional<NEDFrameECEFStepOptions> NEDFrameECEFStepOpts;
        std::optional<FRDFrameECEFStepOptions> FRDFrameECEFStepOpts;
        std::optional<FRDFrameNEDStepOptions> FRDFrameNEDStepOpts;
        std::optional<STABFrameFRDStepOptions> STABFrameFRDStepOpts;
        std::optional<WINDFrameSTABStepOptions> WINDFrameSTABStepOpts;

        static void _validate(const StepOptions& opts);
        // void _clear() noexcept;
    };

    struct _StepOptions {
        // Standard
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

        // Geometric
        std::optional<geography::Latitude> lat;
        std::optional<geography::Longitude> lon;
        std::optional<geography::Altitude> alt;

        // Aerodynamic
        std::optional<aerodynamics::AngleOfAttack> alpha;
        std::optional<aerodynamics::SideslipAngle> beta;

        // Containers
        std::optional<dynamics::RigidBodyState> rbs;
        std::optional<geography::GeographicState> gps;
        std::optional<aerodynamics::AerodynamicState> ads;


        static void _validate(const frames::Frame& F, const _StepOptions& opts);
        explicit operator bool() const;
    };


    struct Aircraft {
        frames::NEDFrameECEF NEDFrameECEF;
        frames::FRDFrameNED FRDFrameNED;
        frames::FRDFrameECEF FRDFrameECEF;
        frames::STABFrameFRD STABFrameFRD;
        frames::WINDFrameSTAB WINDFrameSTAB;
        structural::StructuralProperties structural_properties;
        aerodynamics::AerodynamicProperties aerodynamic_properties;
        control::ControlProperties control_properties;

        Aircraft(
            const structural::StructuralProperties& structural_properties,
            const aerodynamics::AerodynamicProperties& aerodynamic_properties,
            const control::ControlProperties& control_properties
        );

        /** @warning The parent of F must be an inertial frame: ECEFFrame or NEDFrameECEF */
        dynamics::RigidBodyState rigidBodyState(const frames::Frame& F);
        
        /** @warning The parent of F must the ECEFFrame */
        geography::GeographicState geographicState(const frames::Frame& F);

        /** @warning The parent of F must be an inertial frame: ECEFFrame or NEDFrameECEF */
        aerodynamics::AerodynamicState aerodynamicState(const frames::Frame& F, const atmospheric::Wind& windB);

        /** @warning The parent of F must the ECEFFrame */
        atmospheric::AtmosphericState atmosphericState(const frames::Frame& F);

        void step(const StepOptions& opts);
        void _step(frames::NEDFrameECEF& F, const _StepOptions& opts);
        void _step(frames::FRDFrameNED& F, const _StepOptions& opts);
        void _step(frames::FRDFrameECEF& F, const _StepOptions& opts);
        void _step(frames::STABFrameFRD& F, const _StepOptions& opts);
        void _step(frames::WINDFrameSTAB& F, const _StepOptions& opts);
        void _step_dependents(frames::Frame& root);
        void _step_dependent(frames::Frame& F);
        void _step_gravity();

        void _init_frames();

        /** @deprecated */
        // void _set(const dynamics::RigidBodyState& rigidBodyState, frames::Frame& F);
        // void _set(const geography::GeographicState& geographicState, frames::Frame& F);

        void print_state(int t, atmospheric::Wind wind);

    };




}
