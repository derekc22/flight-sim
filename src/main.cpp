#include <string>
#include <spdlog/spdlog.h>
#include "simulation/runner/public/data/types.hpp"
#include "simulation/runner/public/runner.hpp"
#include "core/json/runner/public/parser.hpp"

int main(int argc, char* argv[]) {
    int nargin = 8;
    if (argc != nargin) { 
        spdlog::error("Invalid arguments: expected {}, got {}", nargin-1, argc-1);
        return 1; 
    }

    std::string aircraft_id = argv[1];
    bool data_flag = std::stoi(argv[2]) == 1;
    bool analysis_flag = std::stoi(argv[3]) == 1;
    bool fast_flag = std::stoi(argv[4]) == 1;
    std::string log_dir_path = argv[5];
    std::string data_dir_path = argv[6];
    std::string report_dir_path = argv[7];

    // aggregate cli flags
    runner::CLIFlags cli_flags {
        .data_flag=data_flag,
        .analysis_flag=analysis_flag,
        .fast_flag=fast_flag
    };

    // create cli options
    runner::CLIOptions cli_options {
        .aircraft_id=aircraft_id,
        .flags=cli_flags,
        .log_dir_path=log_dir_path,
        .data_dir_path=data_dir_path,
        .report_dir_path=report_dir_path,
    };

    // create json options
    runner::JSONOptions json_options = json::parse_runner_config();

    // build runner
    runner::Runner runner{cli_options, json_options};

    // run simulation
    runner.run();

    return 0;
}
