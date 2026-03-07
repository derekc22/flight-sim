#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <unordered_map>
#include "simulation/structural/structural.hpp"
#include <simulation/dynamics/dynamics.hpp>

namespace aerodynamics {

    struct Vinf;
    struct LiftCoefficient;
    struct DragCoefficient;
    struct MomentCoefficient;
    struct AngleOfAttack;
    struct SideslipAngle;


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


    std::pair<dynamics::Force, dynamics::Moment> step_aero_forces_moments(const AerodynamicProperties& aero, const structural::StructuralProperties& structural, const dynamics::RigidBodyState& rbs, double rho);



    struct Vinf {
        double data; // freestream velocity, V∞​ [ms^-1]
    };

    struct LiftCoefficient {
        double data;    // [dimensionless]
    };

    struct DragCoefficient {
        double data;    // [dimensionless]
    };

    struct MomentCoefficient {
        double data;    // [dimensionless]
    };

    struct AngleOfAttack {
        double data;    // [rad]
    };

    struct SideslipAngle {
        double data;    // [rad]
    };

    struct AerodynamicState {
        Vinf vinf;    
        AngleOfAttack alpha;   
        SideslipAngle beta;
    };


    AerodynamicState compute_aerodynamic_state(const dynamics::RigidBodyState& rbs);


    dynamics::OrientationMatrix CBS(const aerodynamics::AngleOfAttack& alpha);
    dynamics::OrientationMatrix CSW(const aerodynamics::SideslipAngle& beta);



}