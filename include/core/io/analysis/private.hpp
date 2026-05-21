#pragma once
#include <string>
#include "simulation/constants/public.hpp"

namespace io {

    struct MatlabContext {
        std::string aircraft_id;
        int state_dim = constants::state_dim;
        int input_dim = constants::input_dim;
        std::string A_csv;
        std::string B_csv;
        std::string C_csv;
        std::string D_csv;
    };

}