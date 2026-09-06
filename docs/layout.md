# C++ Layout Rules

These rules define the final layout contract for C++ modules under `include/simulation`, `src/simulation`, `include/core`, and `src/core`.

## Visibility

- `public/` contains declarations that code outside the module may use.
- `private/` contains algorithms and implementation types used only by the owning module.
- Under `src/`, `public/` contains definitions declared by public headers and `private/` contains definitions declared by private headers.
- Source visibility names classify the declarations being implemented; source files themselves are never included as interfaces.
- A source file's placement follows the declarations it defines, not its dependencies; a public implementation may use private headers from its own module.
- Public headers and public template definitions must never include private headers.
- Production code must not include another module's private headers.
- Tests may include private headers for deliberate white-box testing.

## Standalone Public Objects

A public behavior-bearing type that is neither shared data, a component, nor a manager is a standalone public object. It is placed directly under `public/` using its semantic name.

```text
include/<area>/<module>/
  public/
    <object>.hpp
    <object>.tpp  # when the object has template definitions

src/<area>/<module>/
  public/
    <object>.cpp  # when the object has out-of-line definitions
```

`<area>` is `simulation` or `core`. A standalone public object represents the thing callers use, such as a vehicle, frame, solver, device, connection, or recorder. Its methods implement that object's own behavior. Internal state, resources, threads, queues, and helper functions are implementation and do not make the object a manager. A standalone object may aggregate subsystem managers when those subsystems are part of the object it represents, as `Aircraft` does.

Use the object's semantic name. If `XManager` would mean a separate object that manages instances of `X`, then `X` itself is a standalone public object and belongs in `public/x.hpp`. A semantic header may contain a cohesive object family, such as the `Frame` hierarchy or the `UDPIn` and `UDPOut` pair; unrelated standalone objects use separate semantic headers.

`frames/public/frame.hpp`, `vehicles/public/aircraft.hpp`, and `qp/public/solver.hpp` are the current simulation examples. Core examples include `connection/public/udp.hpp`, `devices/public/joystick.hpp`, `io/analysis/public/analysis_recorder.hpp`, `io/data/public/data_recorder.hpp`, and `io/rerun/public/rerun_recorder.hpp`.

A standalone public object may coexist with the module's applicable public operations, `data/`, `detail/`, or a distinct subsystem manager.

## Public Operation Interfaces

A module whose public API includes a cohesive free-function operation places that interface directly under `public/` using the operation's semantic name.

```text
include/<area>/<module>/
  public/
    <operation>.hpp
    <operation>.tpp  # when the operation has template definitions

src/<area>/<module>/
  public/
    <operation>.cpp  # when the operation has out-of-line definitions
```

Public operation interfaces are first-class module capabilities, not subordinate implementation functions. They are not data helpers unless their purpose is representation assembly, and subordinate module behavior belongs in `detail`. Current examples include `settings/public/application.hpp`, `trim/public/application.hpp`, `trim/public/inspection.hpp`, and `io/camera/public/capture.hpp`. Core parsers, writers, and file interfaces are specialized public operations with conventional names.

A module may expose both standalone public objects and public operations when both roles exist.

## Reading Layout Trees

The trees below enumerate permitted role locations, not mandatory scaffolding. Create a directory or file only when the module contains code with that role; do not create empty categories to reproduce the full tree. A `.cpp` exists only when its header has out-of-line definitions, and a `.tpp` exists only when its header has template definitions.

A module with components necessarily has at least one component and a manager that directly or transitively owns or selects its component hierarchy. All other displayed roles are conditional. A module without components has no component hierarchy; its data, detail, manager, standalone-object, and public-operation roles are present only when applicable.

## Modules With Components

```text
include/simulation/<module>/
  public/
    <object>.hpp         # when a standalone public object exists
    <object>.tpp         # optional
    <operation>.hpp      # when a public operation exists
    <operation>.tpp      # optional
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

src/simulation/<module>/
  public/
    <object>.cpp         # when the object has out-of-line definitions
    <operation>.cpp      # when the operation has out-of-line definitions
    data/
    components/
    manager.cpp
  private/
    data/
    detail/
    components/
```

Components may use short semantic subdirectories when genuine categories exist, such as `surface/`, `propulsor/`, `air_data/`, or `navigation/`. Do not add a directory layer that would contain only a redundant repetition of the component name.

Control divides its private components into two genuine categories:

```text
private/components/
  controllers/
    <control-domain>/
      <implementation>.hpp
  policies/
    <policy-family>/
      <implementation>.hpp
```

`controllers/` contains implementations of Control's public attitude, velocity, and linear-quadratic components. `policies/` contains behavior-bearing control-law components used by those controllers and their implementation support. The `ControlManager` owns both categories transitively through its component hierarchy. These are Control-specific component categories, not repository-wide roles.

Runner groups `ControlWrapper`, `EstimationWrapper`, `LinearizationWrapper`, `MeasurementsWrapper`, `PhysicsWrapper`, `RecordingWrapper`, and `TrimWrapper` under `public/components/wrappers/`. `Scheduler` and `FlightGearAdapter` remain directly under `public/components/` because they are individual components that do not form a second shared category. `wrappers/` is a Runner-specific component category, not a repository-wide role.

## Modules Without Components

```text
include/simulation/<module>/
  public/
    <object>.hpp         # when a standalone public object exists
    <object>.tpp         # optional
    <operation>.hpp      # when a public operation exists
    <operation>.tpp      # optional
    data/
      types.hpp
      helpers.hpp        # optional
      helpers.tpp        # optional
    detail/
      <function>.hpp
      <function>.tpp  # optional
    manager.hpp          # when a manager exists
    manager.tpp          # optional
  private/
    data/
      types.hpp          # optional
      helpers.hpp        # optional
      helpers.tpp        # optional
    detail/
      <implementation>.hpp

src/simulation/<module>/
  public/
    <object>.cpp         # when the object has out-of-line definitions
    <operation>.cpp      # when the operation has out-of-line definitions
    data/
    detail/
    manager.cpp          # only when a manager exists
  private/
    data/
    detail/
```

## Shared Support Modules

Constants, foundational aliases, and general-purpose functions that have no owning domain module and are intended for use by otherwise unrelated modules belong to shared support modules. They use subject-based filenames under `public/` rather than manager, component, or detail roles. Domain-specific types, constants, aliases, and calculations remain with their owning module.

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
  public/
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

Contains domain-owned shared value types, state structures, payloads, parameters, enums, constants, and aliases.

Intrinsic value-object behavior may remain on its owning type. Behavior is intrinsic when it queries or modifies that type's own representation, enforces its invariants, or validates an instance of that type without coordinating module responsibilities or calculating relationships between independent domain objects. Examples include orientation accessors and conversions, `DataMatrix::insert`, and `StepOptions::validate`.

Constants owned by one domain remain in that domain's data. Constants with no domain owner that are used throughout the codebase belong in the shared `constants` module.

### Public operations

Contains cohesive free-function capabilities exposed as part of a module's public API. The semantic filename names the operation, such as `application.hpp`, `inspection.hpp`, or `capture.hpp`. Representation assembly belongs in `data/helpers`, and domain calculations belong in `detail`.

### `data/helpers`

Contains operations whose primary purpose is to assemble, decompose, pack, unpack, format, or convert shared data representations. This includes selecting or calculating field values needed to construct a state, payload, result, or report representation. A function whose primary result is a domain calculation rather than an assembled data representation belongs in `detail`. Its name may use `make_*`, `build_*`, or another semantic verb; the name alone does not determine its role. `atmospheric::build_wind` is a data helper because it assembles a `Wind` representation from external wind inputs.

### `detail`

Contains subordinate module behavior implemented as functions. It does not contain standalone objects, managers, components, shared type declarations, or representation helpers. Domain calculations and decisions, validation beyond one value type's own invariants, streaming, filesystem support, and similar behavior may all belong in `detail` when they are subordinate to the module's public interfaces. A detail function may construct or extract a mathematical value as part of its calculation; that does not make it a data helper. The SE(3) functions in `transforms/public/detail/se3.hpp` are transform calculations even when they construct or extract matrices. Forward declarations of types owned elsewhere may appear when required by function declarations.

This definition is identical for `public/detail` and `private/detail`; visibility is the only distinction. Public detail functions are required outside the module or by public template definitions. Private detail functions are required only by the owning module. Detail calculations must consume data directly and must not depend on their module's manager.

A generally reusable function with no domain owner belongs in `util`, not in a domain module's detail directory. A function shared by multiple components belongs in detail. Detail files use names that identify their operation; the generic `helpers.hpp` and `helpers.tpp` names are reserved for data helpers.

### `components`

Contains one file per distinct functional responsibility directly or transitively owned or selected within a manager's component hierarchy. A component may own its own state, behavior, and subordinate components. Subcomponent describes an ownership relationship, not a separate layout category. Individual formulas, repeated data entries, and intermediate stages of one calculation are not components. `base.hpp` exists only when components share behavior. `collection.hpp` exists only when a named aggregate of components is useful.

An algorithm used by only one component belongs under `private/components/<owner>/`. Component subdirectories may distinguish genuine categories of components; they do not establish new repository-wide file roles.

### `manager`

Represents the management layer for a simulation subsystem or application execution. A manager coordinates that responsibility through its component hierarchy, subordinate managers, or cohesive subsystem step. It owns the managed responsibility's persistent state and, when applicable, component selection, input-payload distribution, and output-payload aggregation. Managers may form a hierarchy: subsystem managers own subsystem responsibilities, while a top-level manager may coordinate those managers as part of application execution. Internal resources and helper functions are implementation, not managed responsibilities. A manager is declared in `public/manager.hpp` and named for the responsibility it manages, such as `ControlManager`, `SensorManager`, or `RunManager`. Subordinate behavior should remain in components or `detail/`.

`step(input) -> output` is the standard per-tick entry point, not the manager's only permitted method. Managers may use named member methods for coherent orchestration phases; simple managers may orchestrate directly in `step()` without artificial helpers. A member method's visibility under the project's struct convention is not a reason to inline it.

An object does not become a manager merely because it organizes its own behavior or owns persistent state or implementation resources. A public type representing the entity, tool, connection, or recorder being used is a standalone public object. A type whose public role is the management layer for a simulation subsystem or application execution is a manager. A cohesive subsystem manager may have no components when splitting its behavior would create artificial responsibilities; `StructuralManager` is the current example.

## Source Mirroring

Source directories mirror the semantic category and visibility of their declarations. `<area>` is `simulation` or `core`, and `<module>` may contain multiple path segments for a nested core module:

```text
include/<area>/<module>/public/data/helpers.hpp
  -> src/<area>/<module>/public/data/helpers.cpp
include/<area>/<module>/public/data/types.hpp
  -> src/<area>/<module>/public/data/types.cpp
include/<area>/<module>/public/components/foo.hpp
  -> src/<area>/<module>/public/components/foo.cpp
include/<area>/<module>/public/components/<category>/foo.hpp
  -> src/<area>/<module>/public/components/<category>/foo.cpp
include/<area>/<module>/public/detail/foo.hpp
  -> src/<area>/<module>/public/detail/foo.cpp
include/<area>/<module>/public/manager.hpp
  -> src/<area>/<module>/public/manager.cpp
include/<area>/<module>/public/<semantic>.hpp
  -> src/<area>/<module>/public/<semantic>.cpp
include/<area>/<module>/private/data/foo.hpp
  -> src/<area>/<module>/private/data/foo.cpp
include/<area>/<module>/private/components/<owner>/foo.hpp
  -> src/<area>/<module>/private/components/<owner>/foo.cpp
include/<area>/<module>/private/components/<category>/<owner>/foo.hpp
  -> src/<area>/<module>/private/components/<category>/<owner>/foo.cpp
include/<area>/<module>/private/detail/foo.hpp
  -> src/<area>/<module>/private/detail/foo.cpp
```

Template definitions remain in the corresponding `.tpp`. Every `.hpp` and `.tpp` uses `#pragma once`.

## Exceptions

- C or Fortran interoperability headers may use the extension required by the interface.

## Core Modules

Core modules follow the same `public/` and `private/` visibility rules without forcing simulation-specific manager or component roles onto infrastructure code.

- Standalone public infrastructure objects follow the standalone-object rule, such as `connection/public/udp.hpp`, `devices/public/joystick.hpp`, `io/analysis/public/analysis_recorder.hpp`, `io/data/public/data_recorder.hpp`, and `io/rerun/public/rerun_recorder.hpp`.
- Shared value types and representation helpers use `public/data/types.hpp` and `public/data/helpers.hpp`.
- IO types follow the same standalone-object and manager rules; persistent state, threads, queues, files, or streams do not by themselves establish a manager role.
- Public operation interfaces follow the general semantic naming rule, such as `io/camera/public/capture.hpp`.
- A JSON domain's `public/parser.hpp` contains its public configuration-to-domain entry points. `private/parsing.hpp` contains subordinate JSON-to-domain construction, and `private/validation.hpp` contains checks of the external JSON representation. These private files exist only when their roles are present.
- A JSON `public/writer.hpp` contains public domain-to-JSON output operations. Shared JSON representation assembly belongs in `json/public/data/helpers.hpp`.
- A module's `public/files.hpp` contains file and path operations exposed to other modules. Its `private/detail/files.hpp` contains subordinate file implementation used only by that module.
- Core source paths mirror visibility and semantic roles: `public/parser.hpp` maps to `public/parser.cpp`, `private/parsing.hpp` to `private/parsing.cpp`, `private/validation.hpp` to `private/validation.cpp`, `public/writer.hpp` to `public/writer.cpp`, and `public/files.hpp` to `public/files.cpp`.

## Required Checks

- Every file has one documented semantic role, and a directory exists only when that role is present.
- No shared structs, enums, or aliases under `detail/`.
- No module calculations under `data/types`.
- No output-assembly helpers under `detail/`.
- Domain-owned calculations remain with their domain; ownerless general-purpose operations belong in shared support.
- No public header or template definition includes `private/`.
- No production module includes another module's `private/` tree.
- No detail function depends on its own manager.
- Standalone public objects and public operations use semantic root-level headers. A type whose public role is the management layer for a simulation subsystem or application execution uses `public/manager.hpp` and the `<Responsibility>Manager` name.
- Component-specific implementations remain with their owner; genuine component categories may use semantic subdirectories. Shared supporting functions use `detail`.
- Source paths mirror declaration visibility and semantic role.
- Core parser, parsing, validation, writer, and file interfaces use their defined semantic paths.
- No obsolete flat `public.hpp`, `private.hpp`, or legacy submodule directories remain after a module is refactored.
- Do not add nested namespaces.
