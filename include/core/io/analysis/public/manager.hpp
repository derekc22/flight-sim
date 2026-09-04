#pragma once
#include "core/io/analysis/public/data/types.hpp"

#include <string>

namespace io
{

	struct AnalysisManager {
		runner::JSONFlags json_flags;

		AnalysisContext context;
		MATLABContext matlab_context;

		AnalysisManager(const std::string& aircraft_id,
			const runner::JSONFlags& json_flags,
			const runner::ModuleRates& module_rates);
		void save(const std::string& data_dir_path, const std::string& report_dir_path);
	};

} // namespace io
