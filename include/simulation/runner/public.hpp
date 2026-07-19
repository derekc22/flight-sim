#pragma once

namespace runner {

	struct CLIFlags {
        bool data_flag;
        bool analysis_flag;
        bool fast_flag;
    };

    struct JSONFlags {
        bool trim_flag;
        bool avionics_flag;
        bool control_flag;
        bool estimation_flag;
        bool wind_flag;
        bool verbose_flag;
        bool rerun_flag;
        bool joystick_flag;
    };

    struct ModuleRates {
        double sensor_hz;
        double avionics_hz;
        double estimation_hz;
        double guidance_hz;
        double control_hz;
        double log_hz;
    };

    struct JSONOptions {
        int tf;
        JSONFlags flags;
        ModuleRates module_rates;
    };
}