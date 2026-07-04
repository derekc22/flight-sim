#pragma once
#include <string>
#include "analysis/eigenanalysis/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/runner/public.hpp"
#include "simulation/constants/public.hpp"

namespace io {

    struct MatlabContext {
        std::string aircraft_id;
        int state_dim = constants::state_dim;
        int input_dim = constants::input_dim;
        std::string A_csv;
        std::string B_csv;
        std::string C_csv;
        std::string D_csv;
    };

    struct AnalysisContext {
        std::string aircraft_id;
        trim::TrimSolution trim_sol;
        linearization::LocalLinearization lin_sol;
        analysis::EigenAnalysis eig_sol;
    };

    struct AnalysisManager {
        runner::CLIFlags cli_flags;
        runner::JSONFlags json_flags;

        std::string aircraft_id;
        trim::TrimSolution trim_sol;
        linearization::LocalLinearization lin_sol;
        analysis::EigenAnalysis eig_sol;

        void step(const AnalysisContext& context);
        void save(const std::string& data_dir_path, const std::string& report_dir_path);
    };
}
