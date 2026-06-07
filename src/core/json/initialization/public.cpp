#include <nlohmann/json.hpp>
#include "core/json/initialization/private.hpp"
#include "core/json/initialization/public.hpp"
#include "core/json/public.hpp"
#include "simulation/vehicles/public.hpp"

namespace json {

    vehicles::StepOptions parse_initialization_config(bool trim_bool) {
        const auto config_path = resolve_run_config_entry_path("initialization_config");
        const auto config = read_json_file(config_path);
        validate_initialization_config(config, trim_bool);

        vehicles::StepOptions opts;
        if (config.contains("NEDFrameECEF")) { opts.NEDFrameECEFStepOpts = parse_NEDFrameECEF_step_options(config.at("NEDFrameECEF")); }
        if (config.contains("FRDFrameECEF")) { opts.FRDFrameECEFStepOpts = parse_FRDFrameECEF_step_options(config.at("FRDFrameECEF")); }
        if (config.contains("FRDFrameNED")) { opts.FRDFrameNEDStepOpts = parse_FRDFrameNED_step_options(config.at("FRDFrameNED")); }
        return opts;
    }

}
