# Flight Sim Refactor Handoff Summary

## Purpose

This file is for the next agent reviewing the `simulation/` layout refactor. The user no longer trusts the previous agent's judgment and wants an independent, careful review. Do not assume this refactor is correct just because the build passes. Verify it.

The review target is the active repo under:

`/Users/derekchibuzor/Documents/VSCodeProjects/flight-sim`

Do not inspect or consider `ame532`. Do not inspect `config/templates`.

## Critical User Rules

- Publicly used symbols belong in `public.*`.
- Module-private symbols belong in `<module>.*`.
- Multi-module areas should use real subdirectories with their own `public.*` and `<module>.*`.
- Preserve every existing namespace, type, alias, function, template, variable, signature, definition, and behavior unless the user explicitly approves a change.
- Do not remove, rewrite, add to, or edit comments during refactors.
- Do not add or use nested namespaces.
- Do not declare C++ functions `const` unless the user explicitly approves.
- Keep function arguments inline when writing functions.
- If unsure how to proceed, stop and ask the user.
- If a required change seems beyond the user's requested scope, stop and ask the user.
- Leave CARE/SLICOT alone unless the user explicitly asks.

CARE/SLICOT paths to avoid:

- `include/simulation/control/linear_quadratic/slicot_care_wrapper.h`
- `src/simulation/control/linear_quadratic/slicot_care_wrapper.f90`

## Desired Architecture

The intended layout is:

- `include/simulation/<module>/public.hpp`: cross-module public declarations.
- `include/simulation/<module>/public.tpp`: public template definitions, where needed.
- `include/simulation/<module>/<module>.hpp`: module-private declarations and module aggregate/private owner.
- `include/simulation/<module>/<module>.tpp`: private template definitions, where needed.
- `src/simulation/<module>/public.cpp`: public source definitions.
- `src/simulation/<module>/<module>.cpp`: private source definitions.

For submodules, apply the same rule locally, for example:

- `include/simulation/actuators/propulsor/public.hpp`
- `include/simulation/actuators/propulsor/propulsor.hpp`
- `src/simulation/actuators/propulsor/public.cpp`
- `src/simulation/actuators/propulsor/propulsor.cpp`

One intentional exception currently exists:

- `include/simulation/control/shared.hpp`

That file is approved by the user. It restores the old `control/interface.hpp` role under the new name `shared.hpp` and prevents public aggregate include cycles in control.

## Conversation Timeline

The user started with a repo-wide complaint: the codebase was incoherent after trying to split public symbols from private module symbols. The user wanted advice, then implementation.

Stage 1 request:

- Migrate old broad `shared.*` files into `public.*` or `<module>.*`.
- Delete old `shared.*` only after preserving symbols.
- Preserve comments and symbols.

Stage 2 request:

- Normalize all of `include/simulation` and mirrored `src/simulation`.
- Add missing `public.hpp` files.
- Restructure multi-module areas like actuators, avionics, control, estimation, transforms, and util into subfolders.
- Delete old flat headers with no forwarding wrappers.

Challenges from the user:

- Avionics appeared to have only public files. The answer given was that the concrete avionics structs are externally constructed/exposed, so their member functions stayed public.
- Transforms still had flat `src/simulation/transforms/{s3,se3,so3}.cpp`. That was admitted as an oversight and later fixed.
- The user rejected a macro workaround named `SIMULATION_CONTROL_PUBLIC_NO_SUBMODULE_INCLUDES`.
- The user requested restoring the old `control/interface.hpp` role under the new name `control/shared.hpp`.
- The user later explicitly said `ame532` is not to be touched or even considered.
- The user then requested one more thorough pass, including include lines, public/private symbol placement, subfolders, and unnecessary includes while not relying on implicit includes.

## Current Implemented Shape

Current active simulation tree has these notable shapes:

- Actuators:
  - `actuators/public.*`, `actuators/actuators.*`
  - `actuators/actuator/public.hpp`, `actuators/actuator/actuator.hpp`
  - `actuators/propulsor/public.*`, `actuators/propulsor/propulsor.*`
  - `actuators/surface/public.*`, `actuators/surface/surface.hpp`
- Avionics:
  - `avionics/public.*`, `avionics/avionics.hpp`
  - `avionics/sensors/public.*`, `avionics/sensors/sensors.hpp`
  - `avionics/computers/public.*`, `avionics/computers/computers.hpp`
- Control:
  - `control/shared.hpp` is intentional.
  - `control/public.*`, `control/control.hpp`
  - `control/pid/public.*`, `control/pid/pid.hpp`
  - `control/pid/controllers/{axial,damper,velocity}/public.*`
  - `control/pid/controllers/{axial,damper,velocity}/{axial,damper,velocity}.hpp`
  - `control/linear_quadratic/public.*`, `control/linear_quadratic/linear_quadratic.*`
  - `control/linear_quadratic/controllers/{lqi,lqr,lqt}/public.*`
  - `control/linear_quadratic/controllers/{lqi,lqr,lqt}/{lqi,lqr,lqt}.hpp`
- Estimation:
  - `estimation/public.*`, `estimation/estimation.hpp`
  - `estimation/kalman/public.*`, `estimation/kalman/kalman.hpp`
  - `estimation/kalman/estimators/{ekf,lkf}/public.*`
  - `estimation/kalman/estimators/{ekf,lkf}/{ekf,lkf}.hpp`
- Transforms:
  - `transforms/public.hpp`, `transforms/transforms.*`
  - `transforms/s3/public.*`, `transforms/s3/s3.*`
  - `transforms/se3/public.*`, `transforms/se3/se3.*`
  - `transforms/so3/public.*`, `transforms/so3/so3.*`
- Util:
  - `util/public.*`, `util/util.hpp`
  - `util/{cppad,linalg,trig,units,validate}/public.*`
  - `util/{cppad,linalg,trig,units,validate}/{cppad,linalg,trig,units,validate}.hpp`

There is no `include/simulation/navigation/public.hpp` at the moment. `navigation.hpp` and `navigation.cpp` appear effectively empty and were treated as not having exported symbols.

## What Changed In The Last Audit Pass

The last pass, immediately before this handoff update, found and fixed real issues. It was not just a no-op verification pass.

Duplicate active includes were removed from:

- `include/core/io/io.hpp`
- `include/simulation/aerodynamics/public.hpp`
- `include/simulation/trim/trim.hpp`
- `src/simulation/control/linear_quadratic/controllers/lqr/public.cpp`
- `src/simulation/control/pid/controllers/axial/public.cpp`
- `src/simulation/control/pid/controllers/velocity/public.cpp`
- `src/simulation/dynamics/public.cpp`
- `src/simulation/frames/public.cpp`

Direct standard-library includes were added where headers had been relying on transitive includes:

- `include/simulation/analysis/analysis.hpp`: `<complex>`, `<string>`
- `include/simulation/constants/public.hpp`: `<cstddef>`
- `include/simulation/transforms/s3/public.hpp`: `<string>`
- `include/simulation/transforms/s3/s3.hpp`: `<string>`, `<vector>`
- `include/simulation/transforms/se3/public.hpp`: `<string>`
- `include/simulation/transforms/se3/se3.hpp`: `<vector>`
- `include/simulation/transforms/so3/public.hpp`: `<string>`
- `include/simulation/transforms/so3/so3.hpp`: `<string>`, `<vector>`
- `include/simulation/vehicles/public.hpp`: `<optional>`
- `include/simulation/trim/trim.hpp`: `<string>`

Source files were adjusted to include their owning headers directly:

- `src/simulation/aerodynamics/aerodynamics.cpp`
- `src/simulation/analysis/analysis.cpp`
- `src/simulation/navigation/navigation.cpp`
- `src/simulation/operating/operating.cpp`
- `src/simulation/transforms/s3/public.cpp`
- `src/simulation/transforms/se3/public.cpp`
- `src/simulation/transforms/so3/public.cpp`
- `src/simulation/transforms/transforms.cpp`

Public definitions were moved into public source files:

- `unpack_full_propulsor_actuator_inputs` moved to `src/simulation/actuators/propulsor/public.cpp`.
- `ActuatorProperties::step(...)` overloads moved to `src/simulation/actuators/public.cpp`.
- `unpack_full_surface_actuator_inputs` moved to `src/simulation/actuators/surface/public.cpp`.
- `linearize_trim_solution` and `print_linearization_solution` moved to `src/simulation/linearization/public.cpp`.
- `inspect_trim`, `print_trim_solution`, `update_state_from_trim`, and `set_control_inputs_from_trim` moved to new `src/simulation/trim/public.cpp`.

Other last-pass changes:

- `src/simulation/actuators/surface/surface.cpp` was deleted after its public definition moved to `surface/public.cpp`.
- Redundant public declarations were removed from `include/simulation/trim/trim.hpp`; those declarations remain in `include/simulation/trim/public.hpp`.
- `analysis::format_complex` definition was changed from having a repeated default argument to `int precision` only. The declaration still has the default argument. This was necessary after `analysis.cpp` began including its owning header.
- `src/simulation/transforms/so3/public.cpp` gained a direct include of `simulation/transforms/s3/public.hpp` because it directly uses `normalize_and_canonicalize`.

## Current Worktree State

The worktree is intentionally very dirty because this refactor moved many files.

Expected categories in `git status --short`:

- Many deleted old flat/shared files.
- Many untracked new `public.*`, submodule folders, and mirrored `src` files.
- Modified core JSON/io include users.
- Modified `CMakeLists.txt`.
- Added `summary.md`.

Do not assume deleted files are accidental. Do not revert them without user approval.

Notable expected deletions include:

- old broad `shared.*` files outside the intentional `include/simulation/control/shared.hpp`
- `include/simulation/control/interface.hpp`
- old flat controller headers and sources
- old flat estimator headers and sources
- old flat transform headers and sources
- old flat util headers and sources

## Verification Already Run

Latest known verification after the final audit pass:

- `cmake -S . -B build`: passed.
- `cmake --build build`: passed.
- Header smoke compile for every `include/simulation/**/*.hpp`: `checked=90 failed=0`.
- Layout shape audit: clean.
- Stale/bad include audit: clean.
- Duplicate active include audit: clean.
- Missing include target audit: clean.
- Private include leak audit: clean.
- Source owning-header audit: clean.
- Direct standard-library include audit: clean.
- Approximate public-definition-outside-`public.cpp` audit: clean.
- Exact symbol-ish inventory stayed at `886` before and after the last pass.

The exact symbol-ish inventory had one missing line and one added line because `analysis::format_complex` no longer repeats the default argument on the definition. That is not a symbol removal.

Known build warning:

- The build emits a SLICOT linker warning that `slicot_care_wrapper.o` was built for newer macOS version `26.0` than linked `14.4`.
- This warning was present during the refactor and was not changed.

## Known Concerns

- The build passing does not prove public/private symbol placement is semantically perfect. The user specifically found a prior transforms oversight by visual inspection, so a human review is required.
- Some comments may mention old paths such as `shared.hpp`. They were intentionally left alone because the user forbids comment edits during refactors.
- `navigation` has no `public.hpp` because it appears empty. Confirm this is acceptable before changing it.
- `include/simulation/control/shared.hpp` is an approved exception, not a repo-wide return to old `shared.*`.
- Public member functions of publicly exposed structs/classes were treated as public symbols and generally belong in `public.cpp`.
- Some module-private source files are now nearly empty after public definitions moved out. That may be acceptable, but should be reviewed.
- `.DS_Store` files exist under simulation directories. They were not touched.
- `docs/misc/tree.txt` is modified but was not central to verification.

## Review Checklist For Next Agent

Start by reading this file, `AGENTS.md`, and the active `git status --short`.

Then verify, without trusting the previous agent:

- No active include references deleted paths like `control/interface.hpp`.
- No `SIMULATION_CONTROL_PUBLIC_NO_SUBMODULE_INCLUDES` remains.
- No non-control `shared.*` layer has been resurrected.
- `include/simulation/control/shared.hpp` is wired as the base control dependency layer.
- Child control public headers include `simulation/control/shared.hpp`, not aggregate `simulation/control/public.hpp`, unless there is a deliberate reason.
- Every public declaration used cross-module is in an owning `public.hpp`.
- Every public definition is in the corresponding `public.cpp` where a `.cpp` definition exists.
- Module-private helpers are in `<module>.hpp` or `<module>.cpp`.
- Public headers do not rely on transitive standard-library includes.
- Source files include their owning header directly.
- Submodules with real independent roles have subfolders rather than flat headers.
- No comments were edited while reviewing.
- No symbols disappeared.

Suggested verification commands:

```sh
git status --short
cmake -S . -B build
cmake --build build
rg -n "SIMULATION_CONTROL_PUBLIC_NO_SUBMODULE_INCLUDES|control/interface\\.hpp" include src CMakeLists.txt
rg -n '#include "simulation/.*/shared\\.(hpp|tpp)"|#include <simulation/.*/shared\\.(hpp|tpp)>' include src CMakeLists.txt
```

For header smoke testing, compile every `include/simulation/**/*.hpp` from `/private/tmp` with the repo include path, Eigen include path, and Homebrew include path. Do not inspect `config/templates`.

For symbol checks, create a pre-review inventory before changing anything and compare after changes. At minimum include lines matching namespace declarations, structs/classes/enums, using declarations, template declarations, functions, and variables. Block completion if a symbol disappears unless the user explicitly approves it.

## Final Note

The latest pass improved the repo, but the user's skepticism is justified. The next agent should treat the build and mechanical audits as useful signals, not as proof. The required review is semantic: are symbols in the right public/private owner files, are submodules shaped correctly, and are includes direct without creating cycles or relying on accidental transitive includes.
