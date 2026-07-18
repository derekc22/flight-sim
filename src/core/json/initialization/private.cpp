#include <stdexcept>
#include <nlohmann/json.hpp>
#include "core/json/initialization/private.hpp"
#include "core/json/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/vehicles/public.hpp"
#include "simulation/util/public.hpp"

namespace json {

    ParsedStepOptions parse_step_options(const nlohmann::json& frame_json) {
        ParsedStepOptions fields;

        if (frame_json.contains("H")) { fields.H = dynamics::HomogeneousTransformationMatrix{ parse_Matrix4d(frame_json.at("H")) }; }
        if (frame_json.contains("C")) { fields.C = dynamics::OrientationMatrix{ parse_Matrix3d(frame_json.at("C")) }; }
        if (frame_json.contains("p")) { fields.p = dynamics::Position{ parse_Vector3d(frame_json.at("p")) }; }
        if (frame_json.contains("q")) { fields.q = dynamics::OrientationQuaternion{ parse_Quaterniond(frame_json.at("q")) }; }
        if (frame_json.contains("eul")) { fields.eul = dynamics::EulerAngles{ parse_Vector3d(frame_json.at("eul")) }; }
        if (frame_json.contains("C_dot")) { fields.C_dot = dynamics::OrientationMatrixRate{ parse_Matrix3d(frame_json.at("C_dot")) }; }
        if (frame_json.contains("q_dot")) { fields.q_dot = dynamics::OrientationQuaternionRate{ parse_Quaterniond(frame_json.at("q_dot")) }; }
        if (frame_json.contains("w")) { fields.w = dynamics::AngularVelocity{ parse_Vector3d(frame_json.at("w")) }; }
        if (frame_json.contains("eul_dot")) { fields.eul_dot = dynamics::EulerAngleRates{ parse_Vector3d(frame_json.at("eul_dot")) }; }
        if (frame_json.contains("wq")) { fields.wq = dynamics::AngularVelocityQuaternion{ parse_Quaterniond(frame_json.at("wq")) }; }
        if (frame_json.contains("v")) { fields.v = dynamics::TranslationalVelocity{ parse_Vector3d(frame_json.at("v")) }; }
        if (frame_json.contains("lat")) { fields.lat = geography::Latitude{ util::deg_to_rad(frame_json.at("lat").get<double>()) }; }
        if (frame_json.contains("lon")) { fields.lon = geography::Longitude{ util::deg_to_rad(frame_json.at("lon").get<double>()) }; }
        if (frame_json.contains("alt")) { fields.alt = geography::GeometricAltitude{ frame_json.at("alt").get<double>() }; }
        if (frame_json.contains("alpha")) { fields.alpha = aerodynamics::AngleOfAttack{ frame_json.at("alpha").get<double>() }; }
        if (frame_json.contains("beta")) { fields.beta = aerodynamics::SideslipAngle{ frame_json.at("beta").get<double>() }; }

        return fields;
    }

    vehicles::NEDFrameECEFStepOptions parse_NEDFrameECEF_step_options(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        return { .lat_NE = fields.lat, .lon_NE = fields.lon, .alt_NE = fields.alt };
    }

    vehicles::FRDFrameECEFStepOptions parse_FRDFrameECEF_step_options(const nlohmann::json& frame_json) {
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

    vehicles::FRDFrameNEDStepOptions parse_FRDFrameNED_step_options(const nlohmann::json& frame_json) {
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

    void validate_NEDFrameECEF_initialization(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        if (!fields.lat.has_value() || !fields.lon.has_value() || !fields.alt.has_value()) { 
            throw std::runtime_error("json::validate_NEDFrameECEF_initialization: lat, lon, alt required"); 
        }
    }

    void validate_FRDFrameECEF_initialization(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        const bool has_H = fields.H.has_value();
        const bool has_C = fields.C.has_value();
        const bool has_p = fields.p.has_value();
        const bool has_q = fields.q.has_value();
        const bool has_eul = fields.eul.has_value();
        const bool has_C_dot = fields.C_dot.has_value();
        const bool has_q_dot = fields.q_dot.has_value();
        const bool has_w = fields.w.has_value();
        const bool has_eul_dot = fields.eul_dot.has_value();
        const bool has_wq = fields.wq.has_value();
        const bool has_v = fields.v.has_value();
        const bool has_lat = fields.lat.has_value();
        const bool has_lon = fields.lon.has_value();
        const bool has_alt = fields.alt.has_value();

        const bool has_geo_all = has_lat && has_lon && has_alt;
        const bool has_position = has_H || has_p || has_geo_all;
        const bool has_orientation = has_H || has_C || has_q || has_eul;
        const bool has_linear_velocity = has_v;
        const bool has_angular_velocity = has_C_dot || has_q_dot || has_w || has_eul_dot || has_wq;

        if (!has_position) { 
            throw std::runtime_error("json::validate_FRDFrameECEF_initialization: one position representation required"); 
        }
        if (!has_orientation) { 
            throw std::runtime_error("json::validate_FRDFrameECEF_initialization: one orientation representation required"); 
        }
        if (!has_linear_velocity) { 
            throw std::runtime_error("json::validate_FRDFrameECEF_initialization: v required"); 
        }
        if (!has_angular_velocity) { 
            throw std::runtime_error("json::validate_FRDFrameECEF_initialization: one angular velocity representation required"); 
        }
    }

    void validate_FRDFrameNED_initialization(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        const bool has_H = fields.H.has_value();
        const bool has_C = fields.C.has_value();
        const bool has_p = fields.p.has_value();
        const bool has_q = fields.q.has_value();
        const bool has_eul = fields.eul.has_value();
        const bool has_C_dot = fields.C_dot.has_value();
        const bool has_q_dot = fields.q_dot.has_value();
        const bool has_w = fields.w.has_value();
        const bool has_eul_dot = fields.eul_dot.has_value();
        const bool has_wq = fields.wq.has_value();
        const bool has_v = fields.v.has_value();

        const bool has_position = has_H || has_p;
        const bool has_orientation = has_H || has_C || has_q || has_eul;
        const bool has_linear_velocity = has_v;
        const bool has_angular_velocity = has_C_dot || has_q_dot || has_w || has_eul_dot || has_wq;

        if (!has_position) { 
            throw std::runtime_error("json::validate_FRDFrameNED_initialization: one position representation required"); 
        }
        if (!has_orientation) { 
            throw std::runtime_error("json::validate_FRDFrameNED_initialization: one orientation representation required"); 
        }
        if (!has_linear_velocity) { 
            throw std::runtime_error("json::validate_FRDFrameNED_initialization: v required"); 
        }
        if (!has_angular_velocity) { 
            throw std::runtime_error("json::validate_FRDFrameNED_initialization: one angular velocity representation required"); 
        }
    }

    void validate_initialization_config(const nlohmann::json& config, bool trim_flag) {
        if (!config.contains("NEDFrameECEF") && !config.contains("FRDFrameECEF")) { 
            throw std::runtime_error("json::validate_initialization_config: One of NEDFrameECEF, FRDFrameECEF required"); 
        }
        if (!config.contains("FRDFrameECEF") && !config.contains("FRDFrameNED")) { 
            throw std::runtime_error("json::validate_initialization_config: One of FRDFrameECEF, FRDFrameNED required"); 
        }
        if (config.contains("WINDFrameSTAB")) { 
            throw std::runtime_error("json::validate_initialization_config: WINDFrameSTAB is an aerodynamic frame. User initialization is not allowed"); 
        }
        if (config.contains("STABFrameFRD")) { 
            throw std::runtime_error("json::validate_initialization_config: STABFrameFRD is an aerodynamic frame. User initialization is not allowed"); 
        }
        if (trim_flag && !config.contains("FRDFrameNED")) { 
            throw std::runtime_error("json::validate_initialization_config: FRDFrameNED required for trim"); 
        }

        if (config.contains("NEDFrameECEF")) { 
            validate_NEDFrameECEF_initialization(config.at("NEDFrameECEF")); 
        }
        if (config.contains("FRDFrameECEF")) { 
            validate_FRDFrameECEF_initialization(config.at("FRDFrameECEF")); 
        }
        if (config.contains("FRDFrameNED")) { 
            validate_FRDFrameNED_initialization(config.at("FRDFrameNED")); 
        }
    }

}
