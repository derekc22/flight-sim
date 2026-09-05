#pragma once
#include "core/io/analysis/public/data/types.hpp"

#include <string>

namespace json
{

	void write_analysis_variables_to_json(const io::AnalysisExportContext& context, const std::string& data_dir_path);

}
