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

		/**
		 * @brief Records and logs the current simulation context.
		 *
		 * @param[in,out] input Step index, simulation data, scheduler, and logging flags.
		 */
		void step(const RecordingWrapperInput& input);

		/**
		 * @brief Saves recorded outputs and configuration snapshots.
		 *
		 * @param[in] cli_options Output paths and enabled recorder options.
		 */
		void cleanup(const CLIOptions& cli_options);
	};

} // namespace runner
