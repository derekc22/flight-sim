#include <nlohmann/json.hpp>
#include <string>
#include "core/io/analysis/private.hpp"
#include "core/json/private.hpp"

namespace json {

    void write_analysis_variables_to_json(const io::MatlabContext& context, const std::string& data_dir_path) {
        nlohmann::json variables = {
            {"aircraft_id", context.aircraft_id},
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
