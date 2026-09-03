#include <Eigen/Eigenvalues>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include "analysis/eigenanalysis/private.hpp"
#include "analysis/eigenanalysis/public.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/linearization/public/data/types.hpp"

namespace analysis {

    EigenAnalysis linearization_eigen_analysis(const linearization::LocalLinearization& lin_sol) {
        Eigen::EigenSolver<linearization::StateJacobian> solver(lin_sol.A);
        if (solver.info() != Eigen::Success) {
            throw std::runtime_error("analysis::linearization_eigen_analysis: Eigenvalue computation failed");
        }

        return EigenAnalysis{
            .eigenvalues = solver.eigenvalues(),
            .eigenvectors = solver.eigenvectors(),
        };
    }

    std::string print_eigen_analysis(const EigenAnalysis& eig) {
        std::ostringstream out;
        for (int mode = 0; mode < constants::state_dim; ++mode) {
            out << "mode " << mode << ":\n";
            out << "lambda: " << format_complex(eig.eigenvalues(mode)) << "\n";
            out << "  vx: " << format_complex(eig.eigenvectors(0, mode)) << "\n";
            out << "  vy: " << format_complex(eig.eigenvectors(1, mode)) << "\n";
            out << "  vz: " << format_complex(eig.eigenvectors(2, mode)) << "\n";
            out << "  p: " << format_complex(eig.eigenvectors(3, mode)) << "\n";
            out << "  q: " << format_complex(eig.eigenvectors(4, mode)) << "\n";
            out << "  r: " << format_complex(eig.eigenvectors(5, mode)) << "\n";
            out << "  phi: " << format_complex(eig.eigenvectors(6, mode)) << "\n";
            out << "  theta: " << format_complex(eig.eigenvectors(7, mode)) << "\n";
            if (mode + 1 < constants::state_dim) out << "\n";
        }
        return out.str();
    }
}
