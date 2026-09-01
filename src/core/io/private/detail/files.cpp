#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include "core/io/private/detail/files.hpp"

namespace io {

    void create_dir(const std::string& dir_path) {
        std::filesystem::create_directories(dir_path);
    }

    void save_vector_to_file(const std::vector<int>& data, const std::string& fname) {
        std::string file_path = "results/data/" + fname + ".csv";
        std::ofstream file_v(file_path);
        for (size_t i = 0; i < data.size(); i++) {
            file_v << data[i];
            if (i < data.size() - 1) file_v << ",";  // comma delimiter
        }
        file_v.close();
    }

    std::string get_datetime() {
        std::time_t std_tm = std::time(nullptr);
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y%b%d_%H-%M-%S", std::localtime(&std_tm));
        std::string dtn = buf;
        return dtn;
    }

}
