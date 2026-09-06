#pragma once
#include "core/io/analysis/public/analysis_recorder.hpp"
#include "core/io/data/public/data_recorder.hpp"
#include "core/io/rerun/public/rerun_recorder.hpp"
#include "simulation/runner/public/data/types.hpp"

#include <optional>

namespace runner
{

	struct RecordingWrapper {
		std::optional<io::DataRecorder> data_recorder;
		std::optional<io::RerunRecorder> rerun_recorder;
		std::optional<io::AnalysisRecorder> analysis_recorder;

		RecordingWrapper(const CLIOptions& cli_options, const JSONOptions& json_options);
		void step(const RecordingWrapperInput& input);
		void cleanup(const CLIOptions& cli_options);
	};

} // namespace runner
