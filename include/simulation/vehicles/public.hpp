#pragma once
#include <optional>
#include <Eigen/Dense>
#include <string>
#include "simulation/frames/public.hpp"
#include "simulation/structural/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/control/public.hpp"
#include "simulation/estimation/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/allocator/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/sensors/public.hpp"
#include "simulation/avionics/public.hpp"
#include "simulation/guidance/public.hpp"



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
        // std::optional<dynamics::HomogeneousTransformationMatrix> HEN;
        // std::optional<dynamics::OrientationMatrix> CEN;
        // std::optional<dynamics::Position> pE_NE;
        // std::optional<dynamics::OrientationQuaternion> qEN;
        // std::optional<dynamics::EulerAngles> eulEN;
        // std::optional<dynamics::OrientationMatrixRate> CEN_dot;
        // std::optional<dynamics::OrientationQuaternionRate> qEN_dot;
        // std::optional<dynamics::AngularVelocity> wN_NE;
        // std::optional<dynamics::EulerAngleRates> eulEN_dot;
        // std::optional<dynamics::AngularVelocityQuaternion> wq_NE;
        // std::optional<dynamics::TranslationalVelocity> vN_NE;
        // std::optional<dynamics::Gravity> gN;
        std::optional<geography::Latitude> lat_NE;
        std::optional<geography::Longitude> lon_NE;
        std::optional<geography::GeometricAltitude> alt_NE;
        std::optional<geography::GeographicState> geo_NE; // container
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
        std::optional<dynamics::HomogeneousTransformationMatrix> HEB; 
        std::optional<dynamics::OrientationMatrix> CEB;
        std::optional<dynamics::Position> pE_BE;
        std::optional<dynamics::OrientationQuaternion> qEB;
        std::optional<dynamics::EulerAngles> eulEB;
        std::optional<dynamics::OrientationMatrixRate> CEB_dot;
        std::optional<dynamics::OrientationQuaternionRate> qEB_dot;
        std::optional<dynamics::AngularVelocity> wB_BE;
        std::optional<dynamics::EulerAngleRates> eulEB_dot;
        std::optional<dynamics::AngularVelocityQuaternion> wq_BE;
        std::optional<dynamics::TranslationalVelocity> vB_BE;
        // std::optional<dynamics::Gravity> gB;
        std::optional<geography::Latitude> lat_BE;
        std::optional<geography::Longitude> lon_BE;
        std::optional<geography::GeometricAltitude> alt_BE;
        std::optional<dynamics::RigidBodyState> X_BE; // container
        std::optional<geography::GeographicState> geo_BE; // container
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
        std::optional<dynamics::HomogeneousTransformationMatrix> HNB; 
        std::optional<dynamics::OrientationMatrix> CNB;
        std::optional<dynamics::Position> pN_BN;
        std::optional<dynamics::OrientationQuaternion> qNB;
        std::optional<dynamics::EulerAngles> eulNB;
        std::optional<dynamics::OrientationMatrixRate> CNB_dot;
        std::optional<dynamics::OrientationQuaternionRate> qNB_dot;
        std::optional<dynamics::AngularVelocity> wB_BN;
        std::optional<dynamics::EulerAngleRates> eulNB_dot;
        std::optional<dynamics::AngularVelocityQuaternion> wq_BN;
        std::optional<dynamics::TranslationalVelocity> vB_BN;
        // std::optional<dynamics::Gravity> gB;
        std::optional<dynamics::RigidBodyState> X_BN; // container
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
        // std::optional<dynamics::HomogeneousTransformationMatrix> HBS; 
        // std::optional<dynamics::OrientationMatrix> CBS;
        // std::optional<dynamics::Position> pB_SB;
        // std::optional<dynamics::OrientationQuaternion> qBS;
        // std::optional<dynamics::EulerAngles> eulBS;
        // std::optional<dynamics::OrientationMatrixRate> CBS_dot;
        // std::optional<dynamics::OrientationQuaternionRate> qBS_dot;
        // std::optional<dynamics::AngularVelocity> wS_SB;
        // std::optional<dynamics::EulerAngleRates> eulBS_dot;
        // std::optional<dynamics::AngularVelocityQuaternion> wq_SB;
        // std::optional<dynamics::TranslationalVelocity> vS_SB;
        // std::optional<dynamics::Gravity> gS;
        std::optional<aerodynamics::AngleOfAttack> alpha;
        std::optional<aerodynamics::AerodynamicState> aero; // container
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
        // std::optional<dynamics::HomogeneousTransformationMatrix> HSW; 
        // std::optional<dynamics::OrientationMatrix> CSW;
        // std::optional<dynamics::Position> pS_WS;
        // std::optional<dynamics::OrientationQuaternion> qSW;
        // std::optional<dynamics::EulerAngles> eulSW;
        // std::optional<dynamics::OrientationMatrixRate> CSW_dot;
        // std::optional<dynamics::OrientationQuaternionRate> qSW_dot;
        // std::optional<dynamics::AngularVelocity> wW_WS;
        // std::optional<dynamics::EulerAngleRates> eulSW_dot;
        // std::optional<dynamics::AngularVelocityQuaternion> wq_WS;
        // std::optional<dynamics::TranslationalVelocity> vW_WS;
        // std::optional<dynamics::Gravity> gW;
        std::optional<aerodynamics::SideslipAngle> beta;
        std::optional<aerodynamics::AerodynamicState> aero; // container
    };

    struct StepOptions {
        std::optional<NEDFrameECEFStepOptions> NEDFrameECEFStepOpts;
        std::optional<FRDFrameECEFStepOptions> FRDFrameECEFStepOpts;
        std::optional<FRDFrameNEDStepOptions> FRDFrameNEDStepOpts;
        std::optional<STABFrameFRDStepOptions> STABFrameFRDStepOpts;
        std::optional<WINDFrameSTABStepOptions> WINDFrameSTABStepOpts;

        static void validate(const StepOptions& opts);
    };

    struct _StepOptions : frames::StandardFrameFieldsOptional {
        // Geometric
        std::optional<geography::Latitude> lat;
        std::optional<geography::Longitude> lon;
        std::optional<geography::GeometricAltitude> alt;

        // Aerodynamic
        std::optional<aerodynamics::AngleOfAttack> alpha;
        std::optional<aerodynamics::SideslipAngle> beta;

        // Containers
        std::optional<dynamics::RigidBodyState> X;
        std::optional<geography::GeographicState> geo;
        std::optional<aerodynamics::AerodynamicState> aero;

        static void validate(const frames::Frame& F, const _StepOptions& opts);
        explicit operator bool() const;
    };


    struct Aircraft {
        std::string id;
        frames::NEDFrameECEF NEDFrameECEF;
        frames::FRDFrameNED FRDFrameNED;
        frames::FRDFrameECEF FRDFrameECEF;
        frames::STABFrameFRD STABFrameFRD;
        frames::WINDFrameSTAB WINDFrameSTAB;
        structural::StructuralProperties structural_properties;
        aerodynamics::AerodynamicProperties aerodynamic_properties;
        actuators::ActuatorProperties actuator_properties;
        control::ControlProperties control_properties;
        sensors::SensorProperties sensor_properties;
        avionics::AvionicsProperties avionics_properties;
        guidance::GuidanceProperties guidance_properties;
        estimation::EstimationProperties estimation_properties;
        allocator::AllocatorProperties allocator_properties;

        Aircraft(
            const std::string& id,
            const structural::StructuralProperties& structural_properties,
            const aerodynamics::AerodynamicProperties& aerodynamic_properties,
            const actuators::ActuatorProperties& actuator_properties,
            const control::ControlProperties& control_properties,
            const sensors::SensorProperties& sensor_properties,
            const avionics::AvionicsProperties& avionics_properties,
            const guidance::GuidanceProperties& guidance_properties,
            const estimation::EstimationProperties& estimation_properties,
            const allocator::AllocatorProperties& allocator_properties
        );

        void step(const StepOptions& opts);
        void step(frames::NEDFrameECEF& F, const _StepOptions& opts);
        void step(frames::FRDFrameNED& F, const _StepOptions& opts);
        void step(frames::FRDFrameECEF& F, const _StepOptions& opts);
        void step(frames::STABFrameFRD& F, const _StepOptions& opts);
        void step(frames::WINDFrameSTAB& F, const _StepOptions& opts);
        void step_dependents(frames::Frame& root);
        void step_dependent(frames::Frame& F);
        void step_gravity();
        void init_frames();
    };

}
