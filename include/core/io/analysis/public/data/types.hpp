#pragma once
#include "analysis/eigenanalysis/public.hpp"
#include "simulation/linearization/public/data/types.hpp"
#include "simulation/runner/public/data/types.hpp"
#include "simulation/trim/public/data/types.hpp"

#include <string>

namespace io
{

	struct AnalysisExportContext {
		std::string aircraft_id;
		runner::ModuleRates module_rates;
		std::string A_csv;
		std::string B_csv;
		std::string C_csv;
		std::string D_csv;
	};

	struct AnalysisContext {
		trim::TrimSolution trim_sol;
		linearization::LocalLinearization lin_sol;
		analysis::EigenAnalysis eig_sol;
	};

} // namespace io
