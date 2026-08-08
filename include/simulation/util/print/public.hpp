#pragma once 
#include <Eigen/Dense>
#include <string>
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/trim/public.hpp"

namespace util {

    std::string print_vec(const char* name, const Eigen::Vector3d& x, const char* unit);

    void print_state(int t, const dynamics::RigidBodyState& Xt, const geography::GeographicState& geo, const aerodynamics::AerodynamicState& aero, const atmospheric::Wind& windI);

}