#pragma once
#include <string>
#include "analysis/eigenanalysis/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/linearization/public/data/types.hpp"
#include "simulation/runner/public/data/types.hpp"
#include "simulation/trim/public/data/types.hpp"

namespace io {

    struct MATLABContext {
        std::string aircraft_id;
        runner::ModuleRates module_rates;
        int state_dim = constants::state_dim;
        int input_dim = constants::input_dim;
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

}
