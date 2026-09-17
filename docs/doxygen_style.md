# Doxygen Style Guide

Use this guide when adding or updating C++ API documentation in the simulation library.

## Scope

- Doxygen function documentation is maintained only for declarations under `include/simulation/`.
- Do not add Doxygen function documentation anywhere outside `include/simulation/`, including `include/core/` and
  `include/analysis/`.
- Do not document constructors, destructors, or deleted copy/move operations.
- Document named functions and non-deleted operators.
- Do not document thin scalar wrappers around standard mathematical or trigonometric operations, including reciprocal
  trigonometric operations such as cosecant, secant, and cotangent.
- Keep documentation for custom mathematical behavior such as unit conversion, smoothing, clamping, and wrapping.
- Do not repeat a documented base-class contract on overriding declarations; document the base declaration only.

## Placement

- Document named-function and non-deleted-operator declarations in header files.
- Do not duplicate the same documentation on matching definitions in `.cpp` files.
- Document private named-function declarations as well as public named-function declarations.
- Place the Doxygen block immediately above the declaration it documents, with no blank line between the block and the declaration.
- Place exactly one blank line between a function declaration and the next Doxygen block.
- Preserve existing non-Doxygen comments exactly unless changing them is explicitly requested.

## Existing Doxygen Documentation

A documentation pass must review every declaration eligible under the scope rules above, including declarations that already have Doxygen comments. The presence of an existing Doxygen block does not mean the function is adequately documented.

- Replace stale, incomplete, inaccurate, or nonstandard Doxygen blocks with documentation that follows this guide.
- Do not preserve an inadequate Doxygen block merely because it already exists.
- Treat an explicit documentation task as authorization to update existing Doxygen comments within the requested scope.
- Preserve unrelated non-Doxygen comments exactly as written.
- Outside an explicit documentation task, do not rewrite Doxygen comments as incidental cleanup.

## Standard Form

```cpp
/**
 * @brief Returns a rotation matrix about the x-axis.
 *
 * The matrix represents a rotation of the coordinate frame by @p angle.
 *
 * @param[in] angle Rotation angle [rad].
 * @return Rotation matrix representing the x-axis rotation [-].
 */
Eigen::Matrix3d Rx(double angle);
```

Use `/** ... */` blocks with one leading `*` on each interior line. End complete sentences with a period.

## Required Content

- Begin with `@brief` followed by one concise sentence describing the function's result or effect.
- Add a normal paragraph after `@brief` only when the function needs more explanation. Do not add `@details`; Doxygen treats subsequent paragraphs as detailed documentation.
- Document parameters in signature order with `@param[in]`, `@param[out]`, or `@param[in,out]`.
- Add `@return` for non-`void` functions. Do not add it to constructors or `void` functions.
- Add `@tparam` for each template parameter.
- Do not use `@pre`, `@post`, or `@throws`.
- Use `@p name` when referring to a parameter in prose.
- Use `@retval` instead of `@return` when individual named return values need separate explanations.

Use `@note` and `@warning` only when they communicate essential behavior that does not belong in the normal description.

## Units

Include units in square brackets after parameter and return descriptions when the quantity has a known unit.

```text
[m]       metre
[s]       second
[kg]      kilogram
[rad]     radian
[rad/s]   radians per second
[m/s]     metres per second
[m/s^2]   metres per second squared
[N]       newton
[N m]     newton metre
[-]       dimensionless
```

Use `[deg]` only when the API explicitly accepts or returns degrees. For generic geometric functions that preserve an unspecified length unit, describe the relationship instead of assuming metres:

```cpp
@param[in] d Translation vector, in the same length units as transformed points.
```

## Mathematical Conventions

Document conventions that affect how a caller interprets the result, including:

- active rotation versus passive coordinate transformation;
- intrinsic versus extrinsic rotation;
- source, destination, and expression frames;
- Euler-axis sequence and angle ordering;
- quaternion component and multiplication conventions;
- normalization or canonicalization;
- multiplication or application order;
- singularities and selected solution ranges.

Do not infer a convention, unit, or behavior from the function name alone. Confirm it from the implementation and existing call sites. If the contract remains ambiguous, ask for clarification before writing the documentation.

## Examples

Short function:

```cpp
/**
 * @brief Returns the angular velocity about the body x-axis.
 *
 * @return Angular velocity component [rad/s].
 */
double p() const;
```

Function with parameters:

```cpp
/**
 * @brief Constructs an active rotation matrix from an extrinsic Euler-angle sequence.
 *
 * @param[in] a First Euler angle in @p order [rad].
 * @param[in] b Second Euler angle in @p order [rad].
 * @param[in] c Third Euler angle in @p order [rad].
 * @param[in] order Euler-axis sequence.
 * @return Active rotation matrix for the Euler-angle sequence [-].
 */
Eigen::Matrix3d eul_to_R(double a, double b, double c, EulerOrder order);
```
