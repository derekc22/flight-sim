#pragma once
#include <string>
#include <Eigen/Dense>
#include "core/io/analysis/public.hpp"
#include "core/io/data/public.hpp"

namespace io {

    void write_txt(const std::string& text, const std::string& dir, const std::string& fname);
    void write_csv(const Eigen::MatrixXd& data, const std::string& dir, const std::string& fname);
}
