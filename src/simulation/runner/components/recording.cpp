#include "core/json/public/files.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/runner/public/components/recording.hpp"
#include "simulation/runner/public/components/scheduler.hpp"
#include "simulation/util/print/public.hpp"

namespace runner {

    Recording::Recording(const CLIOptions& cli_options, const JSONOptions& json_options) {
        // create data manager
        if (cli_options.flags.data_flag) {
            data_manager.emplace(json_options.tf, json_options.flags);
        }
        if (json_options.flags.rerun_flag) {
            rerun_manager.emplace(json_options.flags, json_options.module_rates.log_hz);
        }

        // create analysis manager
        if (cli_options.flags.analysis_flag) {
            // initialize analysis context
            analysis_manager.emplace(cli_options.aircraft_id, json_options.flags, json_options.module_rates);
        }
    }

    void Recording::step(const RecordingInput& input) {
        // update data context
        io::DataContext data_context{
            .Xt=input.context.Xt,
            .Yt=input.context.Yt,
            .Zt=input.context.Zt,
            .u_surface=input.context.u_actual.surface_inputs,
            .u_propulsor=input.context.u_actual.propulsor_inputs,
            .u_surface_commanded=input.context.u_cmd.surface_inputs,
            .u_propulsor_commanded=input.context.u_cmd.propulsor_inputs,
            .WB_net=input.context.WB_net,
            .WB_aerodynamic=input.context.WB_aerodynamic,
            .WB_propulsive=input.context.WB_propulsive,
            .setpoint=input.context.setpoint,
            .windB=input.context.windB
        };

        // step data manager
        if (data_manager) {
            data_manager->step(input.t, data_context);
        }

        if (input.scheduler.log_tick >= constants::hz) {
            // step rerun manager
            if (rerun_manager) { 
                rerun_manager->step(input.t, data_context); 
            }

            // log state
            if (input.flags.verbose_flag) {
                util::print_state(input.t, input.context.Xt, input.context.geo_t, input.context.aero_t, input.context.windI);
            }
            input.scheduler.log_tick -= constants::hz;
        }
    }

    void Recording::cleanup(const CLIOptions& cli_options) {
        std::string data_dir_path = cli_options.data_dir_path;
        std::string log_dir_path = cli_options.log_dir_path;
        std::string report_dir_path = cli_options.report_dir_path;

        // dump configs
        json::dump_run_configs(log_dir_path);

        // save data
        if (data_manager) {
            data_manager->save(data_dir_path);
        }

        // save analysis data
        if (analysis_manager) {
            analysis_manager->save(data_dir_path, report_dir_path);
            json::dump_analyze_configs(log_dir_path);
        }
    }

}
