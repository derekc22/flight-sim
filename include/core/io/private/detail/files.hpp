#pragma once
#include <string>
#include <vector>

namespace io
{

	void create_dir(const std::string& dir_path);
	void save_vector_to_file(const std::vector<int>& data, const std::string& fname);
	std::string get_datetime();
} // namespace io
