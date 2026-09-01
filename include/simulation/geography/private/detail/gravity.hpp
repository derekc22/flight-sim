#pragma once
#include "simulation/dynamics/public/data/types.hpp"

namespace geography {

    /** @brief Given the position of an object WRT the ECEF frame, expressed in the ECEF frame (pE), this function returns the local gravity vector at that position expressed in the ECEF frame (gE) 
        Get local ECEF gravity vector expressed in the ECEF frame based on your current position WRT the ECEF frame expressed in the ECEF frame
    */
    dynamics::Gravity gE(const dynamics::Position& pE);

}
