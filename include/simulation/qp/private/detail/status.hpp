#pragma once
#include "simulation/qp/public/data/types.hpp"

#include <proxsuite/proxqp/dense/dense.hpp>

namespace qp
{

	Status map_status(proxsuite::proxqp::QPSolverOutput status);

}
