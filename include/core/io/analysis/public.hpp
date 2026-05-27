#pragma once
#include <string>
#include "analysis/eigenanalysis/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/trim/public.hpp"

namespace io {

    struct AnalysisContext {
        std::string aircraft_id;
        trim::TrimSolution trim_sol;
        linearization::LocalLinearization lin_sol;
        analysis::EigenAnalysis eig_sol;
    };

    struct AnalysisManager {
        bool data_bool;
        bool analysis_bool;
        bool trim_bool;

        std::string aircraft_id;
        trim::TrimSolution trim_sol;
        linearization::LocalLinearization lin_sol;
        analysis::EigenAnalysis eig_sol;

        void step(const AnalysisContext& context);
        void save(const std::string& data_dir_path, const std::string& report_dir_path);
        AnalysisManager(bool data_bool, bool analysis_bool, bool trim_bool);

    };
}
