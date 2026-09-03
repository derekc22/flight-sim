#pragma once 
#include <Eigen/Dense>
#include <string>
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/geography/public/data/types.hpp"
#include "simulation/aerodynamics/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"

namespace util {

    std::string print_vec(const char* name, const Eigen::Vector3d& x, const char* unit);

    void print_state(int t, const dynamics::RigidBodyState& Xt, const geography::GeographicState& geo, const aerodynamics::AerodynamicState& aero, const atmospheric::Wind& windI);

}
