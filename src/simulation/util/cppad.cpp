#include <stdexcept>
#include "simulation/constants/public/scalars.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/util/public/cppad.hpp"

namespace util {

	CppAD::AD<double> csc(
	    CppAD::AD<double> x)
	{
		return CppAD::AD<double>(1.0) / CppAD::sin(x);
	}

	CppAD::AD<double> sec(
	    CppAD::AD<double> x)
	{
		return CppAD::AD<double>(1.0) / CppAD::cos(x);
	}

	CppAD::AD<double> cot(
	    CppAD::AD<double> x)
	{
		return CppAD::cos(x) / CppAD::sin(x);
	}

	CppAD::AD<double> sin(
	    const CppAD::AD<double>& x)
	{
		return CppAD::sin(x);
	}

	CppAD::AD<double> cos(
	    const CppAD::AD<double>& x)
	{
		return CppAD::cos(x);
	}

	CppAD::AD<double> tan(
	    const CppAD::AD<double>& x)
	{
		return CppAD::tan(x);
	}

	CppAD::AD<double> asin(
	    const CppAD::AD<double>& x)
	{
		return CppAD::asin(x);
	}

	CppAD::AD<double> acos(
	    const CppAD::AD<double>& x)
	{
		return CppAD::asin(x);
	}

	CppAD::AD<double> atan2(
	    const CppAD::AD<double>& y,
	    const CppAD::AD<double>& x)
	{
		return CppAD::atan2(y, x);
	}

	CppAD::AD<double> sqrt(
	    const CppAD::AD<double>& x)
	{
		return CppAD::sqrt(x);
	}

	CppAD::AD<double> abs(
	    const CppAD::AD<double>& x)
	{
		return CppAD::abs(x);
	}

	CppAD::AD<double> smooth_abs(
	    const CppAD::AD<double>& x)
	{
		const CppAD::AD<double> eps_t(constants::eps);
		return sqrt(x * x + eps_t * eps_t) - eps_t;
	}

	CppAD::AD<double> clamp(
	    const CppAD::AD<double>& x,
	    double min_value,
	    double max_value)
	{
		if (max_value < min_value) {
			throw std::runtime_error("util::clamp: max_value must be greater than or equal to min_value");
		}
		if (max_value == min_value)
			return CppAD::AD<double>(min_value);

		const CppAD::AD<double> min_t(min_value);
		const CppAD::AD<double> max_t(max_value);

		return CppAD::CondExpGt(x, max_t, max_t, CppAD::CondExpLt(x, min_t, min_t, x));
	}

	CppAD::AD<double> clamp_symmetric(
	    const CppAD::AD<double>& x,
	    double max_abs)
	{
		if (max_abs <= 0.0)
			return CppAD::AD<double>(0.0);

		const CppAD::AD<double> max_t(max_abs);

		return CppAD::CondExpGt(x, max_t, max_t, CppAD::CondExpLt(x, -max_t, -max_t, x));
	}

	CppAD::AD<double> clamp_positive(
	    const CppAD::AD<double>& x,
	    double max_value)
	{
		if (max_value <= 0.0)
			return CppAD::AD<double>(0.0);

		const CppAD::AD<double> max_t(max_value);

		return CppAD::CondExpLt(
		    x, CppAD::AD<double>(0.0), CppAD::AD<double>(0.0), CppAD::CondExpGt(x, max_t, max_t, x));
	}

	CppAD::AD<double> clamp_to_1(
	    const CppAD::AD<double>& x)
	{
		return CppAD::CondExpGt(x,
		    CppAD::AD<double>(1.0),
		    CppAD::AD<double>(1.0),
		    CppAD::CondExpLt(x, CppAD::AD<double>(-1.0), CppAD::AD<double>(-1.0), x));
	}

	constants::MatrixX_T<CppAD::AD<double>, 3, 1> norm(
	    const constants::MatrixX_T<CppAD::AD<double>, 3, 1>& v)
	{
		const CppAD::AD<double> n = v.norm();

		const CppAD::AD<double> denom =
		    CppAD::CondExpLt(n, CppAD::AD<double>(constants::eps), CppAD::AD<double>(1.0), n);

		const CppAD::AD<double> scale = CppAD::CondExpLt(
		    n, CppAD::AD<double>(constants::eps), CppAD::AD<double>(0.0), CppAD::AD<double>(1.0) / denom);

		return v * scale;
	}

} // namespace util
