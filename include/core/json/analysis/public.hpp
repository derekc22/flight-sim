#pragma once
#include <string>
#include "core/io/analysis/private.hpp"

namespace json {

    void write_analysis_variables_to_json(const io::MatlabContext& context, const std::string& data_dir_path);

}