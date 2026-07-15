#pragma once
#include <string>
#include "core/io/analysis/public.hpp"

namespace json {

    void write_analysis_variables_to_json(const io::MATLABContext& context, const std::string& data_dir_path);

}
