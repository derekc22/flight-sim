#include <casadi/casadi.hpp>
#include <iostream>
#include "simulation/trim/trim.hpp"


namespace trim {

    void test_casadi() {
        casadi::SX x = casadi::SX::sym("x");
        casadi::SX y = x*x;

        casadi::Function f("f",{x},{y});

        std::cout << f(casadi::DM(3))[0] << std::endl;
    }



}