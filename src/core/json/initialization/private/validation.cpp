#include <stdexcept>
#include <nlohmann/json.hpp>
#include "core/json/initialization/private/parsing.hpp"
#include "core/json/initialization/private/validation.hpp"

namespace json {

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
