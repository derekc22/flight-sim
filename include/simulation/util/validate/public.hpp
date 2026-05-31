#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include <string>

namespace util {

    template <typename T>
    void validate_shape(const Eigen::DenseBase<T>& M, Eigen::Index rows, Eigen::Index cols, const std::string& context, const std::string& name);

    template <typename T>
    void validate_square(const Eigen::DenseBase<T>& M, const std::string& context, const std::string& name);

    template <typename T>
    void validate_value(T a, T b, const std::string& context, const std::string& name_a, const std::string& name_b);

}

#include "simulation/util/validate/public.tpp"
