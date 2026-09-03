#pragma once

namespace transforms {

	enum class EulerOrder { ZYX, ZXY, YZX, YXZ, XZY, XYZ, ZYZ, ZXZ, YZY, YXY, XZX, XYX };

	enum class RotationType { Extrinsic, Intrinsic };

	enum class TransformationOrder { RotateFirst, TranslateFirst };

} // namespace transforms
