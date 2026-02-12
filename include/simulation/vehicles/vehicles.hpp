#pragma once
#include <Eigen/Dense>
#include "simulation/frames/frames.hpp"



namespace vehicles {


    struct Aircraft {
        const frames::FRDFrame& BODY;
        double mass;

    };




}