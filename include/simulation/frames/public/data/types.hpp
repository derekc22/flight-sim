#pragma once
#include <optional>
#include "simulation/dynamics/public/data/types.hpp"

namespace frames {

	enum class FrameID { ECEFFrame, NEDFrameECEF, FRDFrameECEF, FRDFrameNED, CGFrameFRD, STABFrameFRD, WINDFrameSTAB };

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

	struct StandardFrameFieldsOptional {
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
	};

	struct SetOptions : StandardFrameFieldsOptional {};

} // namespace frames
