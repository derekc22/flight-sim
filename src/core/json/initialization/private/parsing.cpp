#include "core/json/initialization/private/parsing.hpp"

#include "core/json/public/data/helpers.hpp"
#include "simulation/aerodynamics/public/data/types.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/geography/public/data/types.hpp"
#include "simulation/util/public/trig.hpp"
#include "simulation/vehicles/public/data/types.hpp"

#include <nlohmann/json.hpp>

namespace json
{

	ParsedStepOptions parse_step_options(
		const nlohmann::json& frame_json)
	{
		ParsedStepOptions fields;

		if (frame_json.contains("H")) {
			fields.H = dynamics::HomogeneousTransformationMatrix{parse_Matrix4d(frame_json.at("H"))};
		}
		if (frame_json.contains("C")) {
			fields.C = dynamics::OrientationMatrix{parse_Matrix3d(frame_json.at("C"))};
		}
		if (frame_json.contains("p")) {
			fields.p = dynamics::Position{parse_Vector3d(frame_json.at("p"))};
		}
		if (frame_json.contains("q")) {
			fields.q = dynamics::OrientationQuaternion{parse_Quaterniond(frame_json.at("q"))};
		}
		if (frame_json.contains("eul")) {
			fields.eul = dynamics::EulerAngles{parse_Vector3d(frame_json.at("eul"))};
		}
		if (frame_json.contains("C_dot")) {
			fields.C_dot = dynamics::OrientationMatrixRate{parse_Matrix3d(frame_json.at("C_dot"))};
		}
		if (frame_json.contains("q_dot")) {
			fields.q_dot = dynamics::OrientationQuaternionRate{parse_Quaterniond(frame_json.at("q_dot"))};
		}
		if (frame_json.contains("w")) {
			fields.w = dynamics::AngularVelocity{parse_Vector3d(frame_json.at("w"))};
		}
		if (frame_json.contains("eul_dot")) {
			fields.eul_dot = dynamics::EulerAngleRates{parse_Vector3d(frame_json.at("eul_dot"))};
		}
		if (frame_json.contains("wq")) {
			fields.wq = dynamics::AngularVelocityQuaternion{parse_Quaterniond(frame_json.at("wq"))};
		}
		if (frame_json.contains("v")) {
			fields.v = dynamics::TranslationalVelocity{parse_Vector3d(frame_json.at("v"))};
		}
		if (frame_json.contains("lat")) {
			fields.lat = geography::Latitude{util::deg_to_rad(frame_json.at("lat").get<double>())};
		}
		if (frame_json.contains("lon")) {
			fields.lon = geography::Longitude{util::deg_to_rad(frame_json.at("lon").get<double>())};
		}
		if (frame_json.contains("alt")) {
			fields.alt = geography::GeometricAltitude{frame_json.at("alt").get<double>()};
		}
		if (frame_json.contains("alpha")) {
			fields.alpha = aerodynamics::AngleOfAttack{frame_json.at("alpha").get<double>()};
		}
		if (frame_json.contains("beta")) {
			fields.beta = aerodynamics::SideslipAngle{frame_json.at("beta").get<double>()};
		}

		return fields;
	}

	vehicles::NEDFrameECEFStepOptions parse_NEDFrameECEF_step_options(
		const nlohmann::json& frame_json)
	{
		const ParsedStepOptions fields = parse_step_options(frame_json);
		return {.lat_NE = fields.lat, .lon_NE = fields.lon, .alt_NE = fields.alt};
	}

	vehicles::FRDFrameECEFStepOptions parse_FRDFrameECEF_step_options(
		const nlohmann::json& frame_json)
	{
		const ParsedStepOptions fields = parse_step_options(frame_json);
		return {
			.HEB = fields.H,
			.CEB = fields.C,
			.pE_BE = fields.p,
			.qEB = fields.q,
			.eulEB = fields.eul,
			.CEB_dot = fields.C_dot,
			.qEB_dot = fields.q_dot,
			.wB_BE = fields.w,
			.eulEB_dot = fields.eul_dot,
			.wq_BE = fields.wq,
			.vB_BE = fields.v,
			.lat_BE = fields.lat,
			.lon_BE = fields.lon,
			.alt_BE = fields.alt,
		};
	}

	vehicles::FRDFrameNEDStepOptions parse_FRDFrameNED_step_options(
		const nlohmann::json& frame_json)
	{
		const ParsedStepOptions fields = parse_step_options(frame_json);
		return {
			.HNB = fields.H,
			.CNB = fields.C,
			.pN_BN = fields.p,
			.qNB = fields.q,
			.eulNB = fields.eul,
			.CNB_dot = fields.C_dot,
			.qNB_dot = fields.q_dot,
			.wB_BN = fields.w,
			.eulNB_dot = fields.eul_dot,
			.wq_BN = fields.wq,
			.vB_BN = fields.v,
		};
	}

} // namespace json
