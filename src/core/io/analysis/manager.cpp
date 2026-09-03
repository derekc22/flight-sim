#include <Eigen/Dense>
#include <string>
#include "core/io/analysis/public/manager.hpp"
#include "core/io/public/files.hpp"
#include "core/json/analysis/public/writer.hpp"
#include "analysis/eigenanalysis/public.hpp"
#include "simulation/linearization/public/data/helpers.hpp"
#include "simulation/trim/public/data/helpers.hpp"

namespace io {

	AnalysisManager::AnalysisManager(
	    const std::string& aircraft_id,
	    const runner::JSONFlags& json_flags,
	    const runner::ModuleRates& module_rates)
	    : json_flags(json_flags)
	{
		matlab_context.aircraft_id = aircraft_id;
		matlab_context.module_rates = module_rates;
	}

	void AnalysisManager::save(
	    const std::string& data_dir_path,
	    const std::string& report_dir_path)
	{
		if (json_flags.trim_flag) {
			write_txt(trim::print_trim_solution(context.trim_sol), report_dir_path, "trim_sol");

			if (context.trim_sol.converged) {
				write_txt(linearization::print_linearization_solution(context.lin_sol), report_dir_path, "lin_sol");

				std::string A_fname = "lin_sol_A";
				std::string B_fname = "lin_sol_B";
				std::string C_fname = "lin_sol_C";
				std::string D_fname = "lin_sol_D";

				write_csv(context.lin_sol.A, data_dir_path, A_fname);
				write_csv(context.lin_sol.B, data_dir_path, B_fname);
				write_csv(context.lin_sol.C, data_dir_path, C_fname);
				write_csv(context.lin_sol.D, data_dir_path, D_fname);

				write_txt(analysis::print_eigen_analysis(context.eig_sol), report_dir_path, "eig_sol");

				matlab_context.A_csv = A_fname + ".csv";
				matlab_context.B_csv = B_fname + ".csv";
				matlab_context.C_csv = C_fname + ".csv";
				matlab_context.D_csv = D_fname + ".csv";
			}
		}

		json::write_analysis_variables_to_json(matlab_context, data_dir_path);
	}
} // namespace io
