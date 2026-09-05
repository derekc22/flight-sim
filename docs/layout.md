# C++ Layout Rules

These rules define the final layout contract for C++ modules under `include/simulation`, `src/simulation`, `include/core`, and `src/core`.

## Visibility

- `public/` contains declarations that code outside the module may use.
- `private/` contains algorithms, implementation types, and policies used only by the owning module.
- Public headers and public template definitions must never include private headers.
- Production code must not include another module's private headers.
- Tests may include private headers for deliberate white-box testing.

## Modules With Components

```text
include/simulation/<module>/
  public/
    data/
      types.hpp
      helpers.hpp        # optional
      helpers.tpp        # optional
    components/
      <component>.hpp
      base.hpp           # optional
      collection.hpp     # optional
    manager.hpp
    manager.tpp          # optional
  private/
    data/
      types.hpp          # optional
      helpers.hpp        # optional
      helpers.tpp        # optional
    components/
      <owner>/
        <implementation>.hpp
    detail/
      <implementation>.hpp
      <implementation>.tpp  # optional
    policies/
      <policy>/
        <implementation>.hpp

src/simulation/<module>/
  data/
  components/
  manager.cpp
  private/
    data/
    detail/
    components/
    policies/
```

Components may use short semantic subdirectories when genuine categories exist, such as `surface/`, `propulsor/`, `air_data/`, or `navigation/`. Do not add a directory layer that would contain only a redundant repetition of the component name.

## Modules Without Components

```text
include/simulation/<module>/
  public/
    data/
      types.hpp
      helpers.hpp        # optional
      helpers.tpp        # optional
    detail/
      <implementation>.hpp
      <calculation>.tpp  # optional
    manager.hpp          # only when orchestration or persistent state exists
    manager.tpp          # optional
  private/
    data/
      types.hpp          # optional
      helpers.hpp        # optional
      helpers.tpp        # optional
    detail/
      <calculation>.hpp

src/simulation/<module>/
  data/
  detail/
  manager.cpp            # only when a manager exists
  private/
    data/
    detail/
```

## Top-Level Orchestrators

Top-level orchestrators compose subsystems but are not themselves subsystem managers. Wrappers isolate orchestration responsibilities and runner-level persistent state without representing independent subsystems.

```text
include/simulation/<orchestrator>/
  public/
    adapters/
      <adapter>.hpp
    data/
      types.hpp
    scheduling/
      scheduler.hpp
    wrappers/
      <wrapper>.hpp
    <orchestrator>.hpp
  private/
    data/
      helpers.hpp        # optional
    detail/
      <implementation>.hpp
      <implementation>.tpp  # optional

src/simulation/<orchestrator>/
  adapters/
    <adapter>.cpp
  scheduling/
    scheduler.cpp
  wrappers/
    <wrapper>.cpp
  private/
    data/
      helpers.cpp        # optional
    detail/
      <implementation>.cpp
  <orchestrator>.cpp
```

`Runner` is the current top-level orchestrator. Its wrappers coordinate subsystem managers and exchange per-step data through explicit payloads and `StepContext`. Scheduling and external-system adapters remain separate from subsystem wrappers.

## Shared Support Modules

Cross-cutting constants and utility functions use semantic filenames under a shared `public/` directory rather than manager, component, or detail roles.

```text
include/simulation/constants/
  public/
    scalars.hpp
    linalg.hpp
    dimensions.hpp

include/simulation/util/
  public/
    math.hpp
    math.tpp
    cppad.hpp
    filters.hpp
    linalg.hpp
    print.hpp
    trig.hpp
    units.hpp
    validation.hpp
    validation.tpp

src/simulation/util/
  math.cpp
  cppad.cpp
  filters.cpp
  linalg.cpp
  print.cpp
  trig.cpp
  units.cpp
```

## File Roles

### `data/types`

Contains shared value types, state structures, payloads, parameters, enums, constants, and aliases.

Intrinsic value-object behavior may remain on its owning type. Examples include representation accessors and conversions such as the methods on dynamics orientation types. Module orchestration and calculations involving independent domain objects do not belong here.

### `data/helpers`

Contains operations that assemble, pack, unpack, format, or otherwise construct data representations. This includes `make_*` functions whose purpose is to assemble an output structure.

### `detail`

Contains module calculations and implementation support that does not belong to data, components, policies, managers, adapters, scheduling, or wrappers. Detail files contain functions, not shared type declarations. A module's detail calculations must consume data directly and must not depend on that module's manager.

Public detail functions are calculations required by other modules or public template definitions. Private detail functions may also provide internal validation, translation, construction, streaming, filesystem, or other implementation support required only by the owning module.

### `components`

Contains one file per distinct functional component. `base.hpp` exists only when components share behavior. `collection.hpp` exists only when a named aggregate of components is useful.

Component-specific private algorithms may live under `private/components/<owner>/`. Shared implementation strategies belong under `private/policies/<policy>/`.

### `manager`

Owns orchestration, persistent module state, component selection, input-payload distribution, and output-payload aggregation. Calculation detail should remain in components or `detail/`.

`step(input) -> output` is the standard per-tick entry point, not the manager's only permitted method. Managers may use named member methods for coherent orchestration phases; simple managers may orchestrate directly in `step()` without artificial helpers. A member method's visibility under the project's struct convention is not a reason to inline it.

## Source Mirroring

Source directories mirror the semantic category of their public or private declarations:

```text
public/data/helpers.hpp       -> src/<module>/data/helpers.cpp
public/data/types.hpp         -> src/<module>/data/types.cpp
public/components/foo.hpp     -> src/<module>/components/foo.cpp
public/adapters/foo.hpp       -> src/<orchestrator>/adapters/foo.cpp
public/scheduling/foo.hpp     -> src/<orchestrator>/scheduling/foo.cpp
public/wrappers/foo.hpp       -> src/<orchestrator>/wrappers/foo.cpp
public/detail/foo.hpp         -> src/<module>/detail/foo.cpp
public/manager.hpp            -> src/<module>/manager.cpp
private/detail/foo.hpp        -> src/<module>/private/detail/foo.cpp
```

Template definitions remain in the corresponding `.tpp`. Every `.hpp` and `.tpp` uses `#pragma once`.

## Exceptions

- A behavior-bearing domain object may have a semantic root-level header when neither `manager`, `component`, nor `detail` describes it. `frames/public/frame.hpp`, `qp/public/solver.hpp`, and `vehicles/public/aircraft.hpp` are the current examples.
- `StructuralManager` has no components because its current behavior is one cohesive responsibility.
- C or Fortran interoperability headers may use the extension required by the interface.

## Core Modules

Core modules follow the same `public/` and `private/` visibility rules without forcing simulation-specific manager or component roles onto infrastructure code.

- Behavior-bearing infrastructure objects use short semantic filenames, such as `connection/public/udp.hpp`, `devices/public/joystick.hpp`, and `io/camera/public/capture.hpp`.
- Shared value types and representation helpers use `public/data/types.hpp` and `public/data/helpers.hpp`.
- IO subsystems with persistent orchestration use `public/data/types.hpp` and `public/manager.hpp`.
- JSON domain entry points use `public/parser.hpp`. Internal construction and validation use `private/parsing.hpp` and `private/validation.hpp` only when those roles exist.
- JSON writers use `public/writer.hpp`; shared JSON representation helpers use `json/public/data/helpers.hpp`; JSON file operations use `json/public/files.hpp` and `json/private/detail/files.hpp`.
- Core source paths mirror their semantic header roles.

## Required Checks

- No shared structs, enums, or aliases under `detail/`.
- No module calculations under `data/types`.
- No output-assembly helpers under `detail/`.
- No public header or template definition includes `private/`.
- No production module includes another module's `private/` tree.
- No detail calculation depends on its own manager.
- No obsolete flat `public.hpp`, `private.hpp`, or legacy submodule directories remain after a module is refactored.
- Do not add nested namespaces.
