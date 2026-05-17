#pragma once
#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include "simulation/util/validate/public.hpp"

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

    template <typename T>
    void validate_value(int a, int b, const std::string& context, const std::string& name_a, const std::string& name_b) {
        if (a != b) {
            throw std::invalid_argument(context + ": " + name_a + " and " + name_b + " must be equal");
        }
    }

}
