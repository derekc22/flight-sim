#pragma once
#include "simulation/constants/public/linalg.hpp"
#include "simulation/frames/public/data/types.hpp"

#include <unordered_set>

namespace frames
{

	struct Frame {
		FrameID id;
		Frame* parent = nullptr;

		Frame(FrameID id, Frame* p);

		/**
		 * @brief Sets the frame transformation and synchronizes its orientation and position representations.
		 *
		 * @param[in] H Homogeneous transformation for the frame.
		 */
		void _set(const dynamics::HomogeneousTransformationMatrix& H);

		/**
		 * @brief Sets the frame orientation matrix and synchronizes its equivalent representations.
		 *
		 * @param[in] C Orientation matrix [-].
		 */
		void _set(const dynamics::OrientationMatrix& C);

		/**
		 * @brief Sets the frame position within its homogeneous transformation.
		 *
		 * @param[in] p Frame position [m].
		 */
		void _set(const dynamics::Position& p);

		/**
		 * @brief Sets the frame orientation quaternion and synchronizes its equivalent representations.
		 *
		 * The quaternion is normalized and canonicalized before storage.
		 *
		 * @param[in] q Orientation quaternion [-].
		 */
		void _set(const dynamics::OrientationQuaternion& q);

		/**
		 * @brief Sets the frame Euler angles and synchronizes its equivalent orientation representations.
		 *
		 * @param[in] eul Intrinsic ZYX Euler angles [rad].
		 */
		void _set(const dynamics::EulerAngles& eul);

		/**
		 * @brief Sets the orientation-matrix rate and synchronizes the angular-rate representations.
		 *
		 * @param[in] C_dot Orientation-matrix rate [1/s].
		 */
		void _set(const dynamics::OrientationMatrixRate& C_dot);

		/**
		 * @brief Sets the orientation-quaternion rate and synchronizes the angular-rate representations.
		 *
		 * @param[in] q_dot Orientation-quaternion rate [1/s].
		 */
		void _set(const dynamics::OrientationQuaternionRate& q_dot);

		/**
		 * @brief Sets angular velocity and synchronizes the angular-rate representations.
		 *
		 * @param[in] w Angular velocity [rad/s].
		 */
		void _set(const dynamics::AngularVelocity& w);

		/**
		 * @brief Sets the Euler-angle rates and synchronizes the angular-rate representations.
		 *
		 * @param[in] eul_dot Intrinsic ZYX Euler-angle rates [rad/s].
		 */
		void _set(const dynamics::EulerAngleRates& eul_dot);

		/**
		 * @brief Sets the angular-velocity quaternion and synchronizes the angular-rate representations.
		 *
		 * @param[in] wq Angular-velocity quaternion [rad/s].
		 */
		void _set(const dynamics::AngularVelocityQuaternion& wq);

		/**
		 * @brief Sets the frame translational velocity.
		 *
		 * @param[in] v Translational velocity [m/s].
		 */
		void _set(const dynamics::TranslationalVelocity& v);

		/**
		 * @brief Sets the gravity vector stored by the frame.
		 *
		 * @param[in] g Gravity vector [m/s^2].
		 */
		void _set(const dynamics::Gravity& g);

		/**
		 * @brief Returns mutable access to the frame's standard fields.
		 *
		 * @return Mutable pointers to the frame's transformation, rates, velocity, and gravity fields.
		 */
		virtual MutableFrameView view() = 0;

		/**
		 * @brief Returns read-only access to the frame's standard fields.
		 *
		 * @return Read-only pointers to the frame's transformation, rates, velocity, and gravity fields.
		 */
		virtual FrameView view() const = 0;

		virtual ~Frame();

		/**
		 * @brief Applies the populated frame fields in a set of options.
		 *
		 * Options are applied in field order. When multiple equivalent representations are populated, later fields can
		 * overwrite values synchronized from earlier fields.
		 *
		 * @param[in] opts Optional frame fields to apply.
		 */
		void set(const SetOptions& opts);

		std::unordered_set<Frame*> dependents;
		std::unordered_set<Frame*> dependent_on;

		/**
		 * @brief Registers another frame as directly dependent on this frame.
		 *
		 * @param[in] p Dependent frame to register.
		 */
		void add_as_direct_dependent(Frame* p);
	};

	struct ECEFFrame : Frame {
		ECEFFrame();
		dynamics::HomogeneousTransformationMatrix H{constants::HI};
		dynamics::OrientationQuaternion q{constants::qI};
		dynamics::EulerAngles eul{constants::Zero3};
		dynamics::OrientationMatrixRate C_dot{constants::Zero3x3};
		dynamics::OrientationQuaternionRate q_dot{constants::q0};
		dynamics::AngularVelocity w{constants::Zero3};
		dynamics::EulerAngleRates eul_dot{constants::Zero3};
		dynamics::AngularVelocityQuaternion wq{constants::q0};
		dynamics::TranslationalVelocity v{constants::Zero3};
		dynamics::Gravity g{constants::Zero3};

		MutableFrameView view() override;

		FrameView view() const override;
	};

	// {ECEF} -> {NED}
	struct NEDFrameECEF : Frame {
		NEDFrameECEF(ECEFFrame* pECEFFrame);
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

	// {ECEF} -> {FRD}
	struct FRDFrameECEF : Frame {
		FRDFrameECEF(ECEFFrame* pECEFFrame);
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

	// {NED} -> {FRD}
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

	// {FRD} -> {CG}
	struct CGFrameFRD : Frame {
		CGFrameFRD(FRDFrameNED* pFRDFrameNED);
		dynamics::HomogeneousTransformationMatrix HBG;
		dynamics::OrientationQuaternion qBG;
		dynamics::EulerAngles eulBG;
		dynamics::OrientationMatrixRate CBG_dot;
		dynamics::OrientationQuaternionRate qBG_dot;
		dynamics::AngularVelocity wG_GB;
		dynamics::EulerAngleRates eulBG_dot;
		dynamics::AngularVelocityQuaternion wq_GB;
		dynamics::TranslationalVelocity vG_GB;
		dynamics::Gravity gG;

		MutableFrameView view() override;

		FrameView view() const override;
	};

	// {FRD} -> {STAB}
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

} // namespace frames
