#include <unsupported/Eigen/MatrixFunctions>
#include <sstream>
#include <string>
#include "simulation/actuators/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/util/cppad/public.hpp"
#include "simulation/vehicles/public.hpp"
#include "simulation/operating/public.hpp"

namespace linearization {

    DiscretizedLocalLinearization discretize(const linearization::LocalLinearization& lin_sol) {
        int nx = lin_sol.A.rows();
        int nu = lin_sol.B.cols();

        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(nx + nu, nx + nu);

        M.block(0, 0, nx, nx) = lin_sol.A;
        M.block(0, nx, nx, nu) = lin_sol.B;

        Eigen::MatrixXd Md = (M * constants::dt).exp();

        Eigen::MatrixXd Ak = Md.block(0, 0, nx, nx);
        Eigen::MatrixXd Bk = Md.block(0, nx, nx, nu);

        // C and D are pass-through
        return { .A = Ak, .B = Bk, .C = lin_sol.C, .D = lin_sol.D };
    }

    DiscretizedLocalLinearization discretize_euler(const linearization::LocalLinearization& lin_sol) {
        int nx = lin_sol.A.rows();

        Eigen::MatrixXd Ak = Eigen::MatrixXd::Identity(nx, nx) + constants::dt * lin_sol.A;
        Eigen::MatrixXd Bk = constants::dt * lin_sol.B;

        // C and D are pass-through
        return { .A = Ak, .B = Bk, .C = lin_sol.C, .D = lin_sol.D };
    }

    LocalLinearization linearize_operating_point(vehicles::Aircraft& aircraft, const operating::OperatingPoint& operating_point, const operating::OperatingConditions& conditions) {
        const trim::TrimModel model{
            .structural = aircraft.structural_properties,
            .aerodynamic = aircraft.aerodynamic_properties,
            .propulsor_actuators = aircraft.actuator_properties.propulsor_actuators,
            .actuator_limits = actuators::pack_actuator_limits(aircraft.actuator_properties.surface_actuators, aircraft.actuator_properties.propulsor_actuators),
            .fixed_actuator_inputs = actuators::FixedActuatorInputs{
                .flap = aircraft.operating_properties.fixed_actuator_inputs.flap,
                .spoiler = aircraft.operating_properties.fixed_actuator_inputs.spoiler,
            },
        };

        const trim::TrimVariablesVector_T<double> z = trim::unpack_trim_variables_T<double>(operating_point.state, operating_point.input);
        CppAD::eigen_vector<CppAD::AD<double>> z_t = util::start_autodiff_tracking(z);

        const trim::TrimVariablesVector_T<CppAD::AD<double>> z_vec = util::eigen_vector_from_cppad_vector<CppAD::AD<double>, trim::trim_variable_dim>(z_t);
        const dynamics::State_T<CppAD::AD<double>> x_t = trim::pack_trim_state_T<CppAD::AD<double>>(z_vec);
        const actuators::ActuatorInputs_T<CppAD::AD<double>> u_t = trim::pack_trim_actuator_inputs_T<CppAD::AD<double>>(z_vec);
        
        const dynamics::StateDot_T<CppAD::AD<double>> state_dot = trim::compute_trim_state_dot_T<CppAD::AD<double>>(x_t, u_t, model, conditions);
        const dynamics::StateDotVector_T<CppAD::AD<double>> x_dot_vec = dynamics::unpack_state_dot_T(state_dot);
        const CppAD::eigen_vector<CppAD::AD<double>> x_dot_t = util::cppad_vector_from_eigen_vector(x_dot_vec);
        
        CppAD::ADFun<double> f(z_t, x_dot_t);
        const Eigen::Matrix<double, constants::state_dim, trim::trim_variable_dim> jac_map = util::compute_jac<constants::state_dim, trim::trim_variable_dim>(f, z);

        LocalLinearization out;
        out.A = jac_map.leftCols<constants::state_dim>();
        out.B = jac_map.rightCols<constants::input_dim>();
        return out;
    }

    std::string print_linearization_solution(const LocalLinearization& lin_sol) {
        std::ostringstream out;
        out << "A:\n" << lin_sol.A << "\n";
        out << "B:\n" << lin_sol.B << "\n";
        return out.str();
    }

}
