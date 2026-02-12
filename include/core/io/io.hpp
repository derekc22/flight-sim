#pragma once
#include <vector>
#include <string>
#include <Eigen/Dense>

namespace io {

    void createDir(const std::string& dir);
    void saveVectorToFile(std::vector<int>& data, std::string fname);


    struct DataMatrix {
        Eigen::MatrixXd data;
        void save(const std::string& dir, const std::string& fname) const;

    };
}
