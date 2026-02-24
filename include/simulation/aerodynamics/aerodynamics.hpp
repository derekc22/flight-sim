#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <unordered_map>
#include "simulation/structural/structural.hpp"
#include <simulation/dynamics/dynamics.hpp>

namespace aerodynamics {

    struct Surface {
        std::string id;

        double chord;
        double span;
        Eigen::Vector3d p_ref;   // reference location from geometry
        Eigen::Vector3d n;
        double CL0, e, i, CD0, CDa, a0, CM0, CMa;

        double area;
        double AR;
        Eigen::Vector3d p_ac;    // quarter-chord aerodynamic center


    };


    struct AerodynamicProperties {
        std::vector<Surface> surfaces;
        std::unordered_map<std::string, size_t> surfaceIDs;

        AerodynamicProperties(std::vector<Surface> s);

        void compute_aero_properties();
        std::unordered_map<std::string, size_t> build_IDs();

    };


    struct AerodynamicState {
        double Vinf;    // V∞​
        double alpha;   // angle of attack [rad]
        double beta;    // sideslip angle [rad]

    };

    std::pair<dynamics::Force, dynamics::Moment> step_aero_forces_moments(const AerodynamicProperties& aero, const structural::StructuralProperties& structural, const dynamics::RigidBodyState& rbs, double rho);


    AerodynamicState compute_aerodynamic_state(const dynamics::RigidBodyState& rbs);








}