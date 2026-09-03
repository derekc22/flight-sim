#pragma once
#include <chrono>
#include "simulation/constants/public/scalars.hpp"
#include "simulation/runner/public/data/types.hpp"

namespace runner {

    struct Scheduler {
        ModuleRates module_rates;

        double sensor_tick = constants::hz;
        double avionics_tick = constants::hz;
        double estimation_tick = constants::hz;
        double guidance_tick = constants::hz;
        double control_tick = constants::hz;
        double log_tick = constants::hz;

        int sensor_elapsed_ticks = 1;
        int avionics_elapsed_ticks = 1;
        int estimation_elapsed_ticks = 1;
        int control_elapsed_ticks = 1;

        int guidance_tf; // total number of guidance steps
        std::chrono::steady_clock::time_point next;

        Scheduler(const ModuleRates& module_rates, int tf);
        void step(const SchedulerInput& input);
    };

}
