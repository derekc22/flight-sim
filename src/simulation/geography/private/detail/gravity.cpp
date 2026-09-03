#include <Eigen/Dense>
#include "simulation/constants/public/scalars.hpp"
#include "simulation/geography/private/detail/gravity.hpp"

namespace geography {

    dynamics::Gravity gE(const dynamics::Position& pE) { 
        return { -constants::g_earth * pE.data.normalized() }; 
    }

}
