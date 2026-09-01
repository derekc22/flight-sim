#pragma once
#include <string>
#include <Eigen/Dense>

namespace io {

    void write_txt(const std::string& text, const std::string& dir_path, const std::string& fname);
    void write_csv(const Eigen::MatrixXd& data, const std::string& dir_path, const std::string& fname);
}
