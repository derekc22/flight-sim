#include <nlohmann/json.hpp>
#include <string>
#include "core/json/analysis/public.hpp"
#include "core/json/private.hpp"
#include "simulation/constants/public.hpp"

namespace json {

    void write_analysis_variables_to_json(const io::MATLABContext& context, const std::string& data_dir_path) {
        nlohmann::json variables = {
            {"aircraft_id", context.aircraft_id},
            {"hz", {
                {"hz", constants::hz },
                {"control_hz", context.module_rates.control_hz },
            }},
            {"state_dim", context.state_dim},
            {"input_dim", context.input_dim},
            {"matrix_paths", {            
                {"A", context.A_csv},
                {"B", context.B_csv},
                {"C", context.C_csv},
                {"D", context.D_csv},
            }}
        };

        write_json(variables, data_dir_path, "variables");
    }
}
