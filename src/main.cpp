#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>
#include "simulation/constants/public.hpp"
#include "simulation/runner/public.hpp"

int main(int argc, char* argv[]) {
    if (argc != 13) { return 1; }

    std::string aircraft_id = argv[1];

    double time_sec;
    try { time_sec = std::stod(argv[2]); }
    catch (const std::exception&) { std::cerr << "invalid TIME_SEC: " << argv[1] << std::endl; return 1; }
    if (!std::isfinite(time_sec) || time_sec <= 0.0) { std::cerr << "TIME_SEC must be finite and non-negative" << std::endl; return 1; }

    bool trim_bool = std::stoi(argv[3]) == 1;
    bool sensor_bool = std::stoi(argv[4]) == 1;
    bool control_bool = std::stoi(argv[5]) == 1;
    bool estimation_bool = std::stoi(argv[6]) == 1;
    bool wind_bool = std::stoi(argv[7]) == 1;
    bool verbose_bool = std::stoi(argv[8]) == 1;
    bool data_bool = std::stoi(argv[9]) == 1;
    std::string data_dir_path = argv[10];
    std::string report_dir_path = argv[11];
    bool analysis_bool = std::stoi(argv[12]) == 1;

    // compute number of simulation steps
    int tf = std::max(1, static_cast<int>(std::ceil(time_sec / constants::dt)));

    // create simulation input
    runner::SimulationOptions sim_options {
        .aircraft_id=aircraft_id,
        .tf=tf,
        .trim_bool=trim_bool,
        .sensor_bool=sensor_bool,
        .control_bool=control_bool,
        .estimation_bool=estimation_bool,
        .wind_bool=wind_bool,
        .verbose_bool=verbose_bool,
        .data_bool=data_bool,
        .data_dir_path=data_dir_path,
        .report_dir_path=report_dir_path,
        .analysis_bool=analysis_bool
    };

    // run case
    runner::SimulationRunner sim_runner{sim_options};
    sim_runner.run();

    return 0;
}
