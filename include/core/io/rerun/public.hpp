#pragma once
#include <rerun.hpp>
#include <vector>
#include "core/io/data/public.hpp"

namespace io {

    struct RerunManager {
        bool rerun_bool;
        bool control_bool;
        bool avionics_bool;
        bool estimation_bool;
        bool wind_bool;
        rerun::RecordingStream rec;
        std::vector<rerun::Vec3D> trajectory;

        void step(int t, const DataContext& context);
        RerunManager(bool rerun_bool, bool control_bool, bool avionics_bool, bool estimation_bool, bool wind_bool);
    };
}
