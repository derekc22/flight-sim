#include <exception>
#include <iostream>
#include <string>
#include "simulation/constants/public.hpp"
#include "simulation/runner/public.hpp"
#include "core/json/runner/public.hpp"

int main(int argc, char* argv[]) {
    if (argc != 7) { return 1; }

    std::string aircraft_id = argv[1];
    bool data_bool = std::stoi(argv[2]) == 1;
    bool analysis_bool = std::stoi(argv[3]) == 1;
    std::string log_dir_path = argv[4];
    std::string data_dir_path = argv[5];
    std::string report_dir_path = argv[6];

    // create cli options
    runner::CLIOptions cli_options {
        .aircraft_id=aircraft_id,
        .data_bool=data_bool,
        .analysis_bool=analysis_bool,
        .log_dir_path=log_dir_path,
        .data_dir_path=data_dir_path,
        .report_dir_path=report_dir_path,
    };

    // create json options
    runner::JSONOptions json_options = json::parse_runner_config();

    // build runner
    runner::RunManager run_manager{cli_options, json_options};

    // run simulation
    run_manager.run();

    return 0;
}
