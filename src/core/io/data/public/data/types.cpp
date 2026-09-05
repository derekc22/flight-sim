#include "core/io/data/public/data/types.hpp"

#include "simulation/constants/public/scalars.hpp"

#include <Eigen/Dense>
#include <stdexcept>

namespace io
{

	DataMatrix::DataMatrix(
		int n_rows,
		int n_cols)
		: n_rows(n_rows), n_cols(n_cols), data(Eigen::MatrixXd::Zero(n_rows, n_cols))
	{
	}

	void DataMatrix::insert(
		int t,
		const Eigen::VectorXd& input)
	{
		if (input.cols() > 1) {
			throw std::runtime_error("DataMatrix::insert Eigen::Matrix passed for 'input', expected Eigen::Vector");
		}
		if (input.rows() > n_cols - 1) {
			throw std::runtime_error(
				"DataMatrix::insert Number of rows in 'input' exceeds number of columns in DataMatrix");
		}
		if (t > n_rows - 1) {
			throw std::runtime_error("DataMatrix::insert Input index 't' exceeds number of rows in DataMatrix");
		}

		data(t, 0) = t * constants::dt;
		Eigen::Index cols_to_copy = data.cols() - 1;
		data.block(t, 1, 1, cols_to_copy) = input.transpose(); // start_row, start_col, block_rows, block_cols
	}

} // namespace io
