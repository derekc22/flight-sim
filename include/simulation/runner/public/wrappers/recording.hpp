#pragma once
#include <optional>
#include "core/io/analysis/public/manager.hpp"
#include "core/io/data/public/manager.hpp"
#include "core/io/rerun/public/manager.hpp"
#include "simulation/runner/public/data/types.hpp"

namespace runner {

	struct RecordingWrapper {
		std::optional<io::DataManager> data_manager;
		std::optional<io::RerunManager> rerun_manager;
		std::optional<io::AnalysisManager> analysis_manager;

		RecordingWrapper(const CLIOptions& cli_options, const JSONOptions& json_options);
		void step(const RecordingWrapperInput& input);
		void cleanup(const CLIOptions& cli_options);
	};

} // namespace runner
