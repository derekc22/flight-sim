#pragma once
#include <rerun.hpp>
#include "core/io/data/public.hpp"

namespace io {

    struct RerunManager {
        bool rerun_bool;
        bool control_bool;
        bool sensor_bool;
        bool estimation_bool;
        bool wind_bool;
        rerun::RecordingStream rec;

        void step(int t, const DataContext& context);
        RerunManager(bool rerun_bool, bool control_bool, bool sensor_bool, bool estimation_bool, bool wind_bool);
    };
}
