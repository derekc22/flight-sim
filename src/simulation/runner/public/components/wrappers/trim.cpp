#include "simulation/runner/public/components/wrappers/trim.hpp"

#include "simulation/trim/public/inspection.hpp"

namespace runner
{

	TrimWrapperOutput TrimWrapper::step(
		const TrimWrapperInput& input)
	{
		trim_sol = trim::inspect_trim(input.aircraft, input.autodiff_model, input.windB);
		return {.trim_sol = trim_sol};
	}

} // namespace runner
