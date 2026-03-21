#include <casadi/casadi.hpp>
#include <iostream>
#include "simulation/autopilot/autopilot.hpp"


namespace autopilot { // to encompass autonomy and trim

    void test_casadi() {
        casadi::SX x = casadi::SX::sym("x");
        casadi::SX y = x*x;

        casadi::Function f("f",{x},{y});

        std::cout << f(casadi::DM(3))[0] << std::endl;
    }

    void trim() {
        casadi::SX x = casadi::SX::sym("x");

    }



}