#pragma once
#include <Eigen/Dense>
#include <tuple>
#include "simulation/frames/public/frame.hpp"

namespace frames {

    /** @brief Performs a coordinate transformation on a vector vA from frame {A} to frame {B}, producing vB */
    Eigen::Vector3d transform_vec(const Eigen::Vector3d& vA, const Frame& A, const Frame& B);

    /** @brief Performs a homogeneous transformation (translation + coordinate transformation) on a point/position vector pA from frame {A} to frame {B}, producing pB
    If you have point expressed in frame A and described by pA, transform_point(pA, A, B) re-expresses that same point in frame B */
    Eigen::Vector3d transform_point(const Eigen::Vector3d& pA, const Frame& A, const Frame& B);

    dynamics::HomogeneousTransformationMatrix H_from_R(const Frame& F, const Frame& R);

    std::tuple<dynamics::TranslationalVelocity, dynamics::AngularVelocity> vel_from_R(const Frame& F, const Frame& R);

}
