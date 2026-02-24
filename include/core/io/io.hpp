#pragma once
#include <vector>
#include <string>
#include <Eigen/Dense>

namespace io {

    void createDir(const std::string& dir);
    void saveVectorToFile(std::vector<int>& data, std::string fname);


    struct DataMatrix {
        Eigen::MatrixXd data;
        int n_rows;
        int n_cols;

        DataMatrix(const Eigen::MatrixXd& d);
        void write_csv(const std::string& dir, const std::string& fname) const;
        void set(int t, const Eigen::VectorXd data, double dt);

    };
}
