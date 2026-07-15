#include <Eigen/Dense>
#include <string>
#include "core/io/analysis/public.hpp"
#include "core/io/public.hpp"
#include "core/json/analysis/public.hpp"
#include "analysis/eigenanalysis/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/constants/public.hpp"

namespace io {

    AnalysisManager::AnalysisManager(const std::string& aircraft_id, const runner::CLIFlags& cli_flags, const runner::JSONFlags& json_flags, const runner::ModuleRates& module_hz) :
         cli_flags(cli_flags), json_flags(json_flags)
    {
        matlab_context.aircraft_id = aircraft_id;
        matlab_context.module_hz = module_hz;
    }

    void AnalysisManager::save(const std::string& data_dir_path, const std::string& report_dir_path) {
        if (cli_flags.analysis_flag) {

            if (json_flags.trim_flag) {
                write_txt(
                    trim::print_trim_solution(context.trim_sol),
                    report_dir_path,
                    "trim_sol"
                );

                if (context.trim_sol.converged) {
                    write_txt(
                        linearization::print_linearization_solution(context.lin_sol),
                        report_dir_path,
                        "lin_sol"
                    );

                    std::string A_fname = "lin_sol_A";
                    std::string B_fname = "lin_sol_B";
                    std::string C_fname = "lin_sol_C";
                    std::string D_fname = "lin_sol_D";

                    write_csv(context.lin_sol.A, data_dir_path, A_fname);
                    write_csv(context.lin_sol.B, data_dir_path, B_fname);
                    write_csv(context.lin_sol.C, data_dir_path, C_fname);
                    write_csv(context.lin_sol.D, data_dir_path, D_fname);

                    write_txt(
                        analysis::print_eigen_analysis(context.eig_sol),
                        report_dir_path,
                        "eig_sol"
                    );

                    matlab_context.A_csv = A_fname + ".csv";
                    matlab_context.B_csv = B_fname + ".csv";
                    matlab_context.C_csv = C_fname + ".csv";
                    matlab_context.D_csv = D_fname + ".csv";
                }
            }

            json::write_analysis_variables_to_json(matlab_context, data_dir_path);
        }
    }
}
