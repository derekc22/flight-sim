#pragma once
#include <proxsuite/proxqp/dense/dense.hpp>
#include "simulation/qp/public/data/types.hpp"

namespace qp {

    Status map_status(proxsuite::proxqp::QPSolverOutput status);

}
