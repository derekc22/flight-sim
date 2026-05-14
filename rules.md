# Simulation Layout Rules

Rules for active `include/simulation` and `src/simulation` code.

## Core Rule

Publicly used symbols belong in `public.*`.

Module-private symbols belong in `<module>.*`.

A symbol is public if code outside its owning module or submodule includes it,
calls it, constructs it, stores it, names it, or needs it to compile a public
interface. A symbol is private only if it is used entirely inside its owning
module or submodule.

## File Roles

These are allowed file roles, not required files:

```text
include/simulation/<module>/public.hpp
include/simulation/<module>/public.tpp
include/simulation/<module>/<module>.hpp
include/simulation/<module>/<module>.tpp
src/simulation/<module>/public.cpp
src/simulation/<module>/<module>.cpp
```

- `public.hpp`: public declarations, types, aliases, constants, and template declarations.
- `public.tpp`: public template definitions declared in `public.hpp`.
- `<module>.hpp`: private declarations, helper types, aliases, constants, and template declarations.
- `<module>.tpp`: private template definitions declared in `<module>.hpp`.
- `public.cpp`: non-template definitions declared in `public.hpp`.
- `<module>.cpp`: non-template definitions declared in `<module>.hpp`.

Create only files that own real declarations or definitions. Omit `public.cpp`
when there are no public non-template definitions. Omit `<module>.hpp` when
there are no private declarations. Omit `.tpp` files when there are no templates.
If a module has no public symbols and no meaningful private implementation, it
should have no files.

Do not keep unnecessary empty files. Empty namespace blocks, include-only
wrappers, and blank source/template files are invalid unless they have a concrete
current purpose.

## Symbol Placement

Put a symbol in `public.*` when another module, `src/main.cpp`, `core`, JSON,
messages, IO, tests, or any non-owning area uses it. Also put it in `public.*`
when it appears in a public signature/data layout, external code must name it,
or it is a public member function of a publicly exposed type.

Put a symbol in `<module>.*` only when the owning module/submodule is the only
user, no outside public header needs it, no external code names it, and it does
not appear in a public signature/data layout.

Public declarations belong in `public.hpp`; public template definitions in
`public.tpp`; public non-template definitions in `public.cpp`.

Private declarations belong in `<module>.hpp`; private template definitions in
`<module>.tpp`; private non-template definitions in `<module>.cpp`.

Private helpers may stay in the module namespace. File placement, not namespace
nesting, determines public vs private. Do not add nested namespaces.

## Submodules

Multi-module areas must use real subfolders. A submodule follows the same
`public.*` and `<submodule>.*` rules as a top-level module.

Example:

```text
include/simulation/actuators/propulsor/public.hpp
include/simulation/actuators/propulsor/propulsor.hpp
src/simulation/actuators/propulsor/public.cpp
src/simulation/actuators/propulsor/propulsor.cpp
```

Use subfolders for independently meaningful parts such as `actuators/propulsor`,
`actuators/surface`, `avionics/sensors`, `avionics/computers`,
`control/pid/controllers/<controller>`,
`control/linear_quadratic/controllers/<controller>`,
`estimation/kalman/estimators/<estimator>`, `transforms/s3`,
`transforms/se3`, `transforms/so3`, and `util/<utility>`.

Do not keep independent submodules as flat files like `controllers/lqr.hpp` or
`transforms/so3.hpp`.

## Includes And Aggregates

Headers must include what they directly use. Do not rely on accidental
transitive includes.

- Public headers may include needed `public.hpp` headers.
- Public headers must not include another module's private `<module>.hpp`.
- Private headers may include their owning `public.hpp` and direct public dependencies.
- Source files must include their owning header directly.
- Standard library headers must be included directly by the file that uses them.
- Do not use broad aggregate headers to hide missing direct includes.

External code should include public headers, for example
`simulation/actuators/propulsor/public.hpp`, not private owner headers such as
`simulation/actuators/propulsor/propulsor.hpp`.

A module-level `public.hpp` can be a used public aggregate. A used aggregate may
contain no declarations after its includes; an unused aggregate wrapper should
be deleted. Do not add forwarding wrappers for deleted flat paths unless the
user explicitly asks for compatibility wrappers.

## Current Exceptions

New exceptions require explicit user approval.

### `include/simulation/control/shared.hpp`

This is the only approved `shared.hpp` under `include/simulation`.

Reason: control child public headers need shared control input/output types
without including `simulation/control/public.hpp`, because that aggregate
includes the child controllers and would create include cycles.

Rules:

- `control/shared.hpp` replaces the old `control/interface.hpp` role.
- Control child public headers should include `simulation/control/shared.hpp`
  when they need shared control interface types.
- Control child public headers should not include `simulation/control/public.hpp`
  to get those shared types.
- No other `shared.hpp`, `shared.tpp`, or `shared.cpp` layer should be added
  under `include/simulation` or `src/simulation`.
- `SIMULATION_CONTROL_PUBLIC_NO_SUBMODULE_INCLUDES` must not return.

### CARE/SLICOT Wrapper

These files do not follow the normal C++ layout:

```text
include/simulation/control/linear_quadratic/slicot_care_wrapper.h
src/simulation/control/linear_quadratic/slicot_care_wrapper.f90
```

Reason: this is a C++/Fortran interoperability boundary for CARE/SLICOT and is
linked through the standalone `control_care` build path.

Do not inspect, edit, move, or rename these files unless the user explicitly
asks.

## Checklist

When creating or refactoring a module:

1. Identify every external symbol and put it in `public.*`.
2. Put private-only symbols in `<module>.*`.
3. Put template definitions in the matching `.tpp`.
4. Put non-template definitions in the matching `.cpp`.
5. Include direct dependencies, not broad aggregates.
6. Make each source file include its owning header directly.
7. Use a real subfolder for independent submodules.
8. Verify no public header includes another module's private header.
9. Verify no deleted old path or non-control `shared.*` path is referenced.
10. Verify no unnecessary empty shell files were created.
11. Smoke compile the new public and private headers independently.

## Refactor Constraints

During structural refactors:

- Preserve existing namespaces, symbols, signatures, definitions, behavior, and
  comments unless the user explicitly approves a change.
- Do not touch CARE/SLICOT files unless explicitly asked.
