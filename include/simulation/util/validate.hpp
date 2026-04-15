#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include <string>

namespace util {

    template <typename T>
    void validate_shape(const Eigen::DenseBase<T>& M, Eigen::Index rows, Eigen::Index cols, const std::string& context, const std::string& name) {
        if (M.rows() != rows || M.cols() != cols) {
            throw std::invalid_argument(context + ": incorrect shape for " + name + ". Must be (" + std::to_string(rows) + " x " + std::to_string(cols) + ")");
        }
    }

    template <typename T>
    void validate_sqaure(const Eigen::DenseBase<T>& M, const std::string& context, const std::string& name) {
        if (M.rows() != M.cols()) {
            throw std::invalid_argument(context + ": " + name + " must be square");
        }
    }

}
