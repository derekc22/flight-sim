# C++ Layout Rules

These rules define the final layout contract for C++ modules under `include/simulation`, `src/simulation`, `include/core`, and `src/core`.

## Visibility

- `public/` contains declarations that code outside the module may use.
- `private/` contains algorithms, implementation types, and policies used only by the owning module.
- Under `src/`, `public/` contains definitions declared by public headers and `private/` contains definitions declared by private headers.
- Source visibility names classify the declarations being implemented; source files themselves are never included as interfaces.
- A source file's placement follows the declarations it defines, not its dependencies; a public implementation may use private headers from its own module.
- Public headers and public template definitions must never include private headers.
- Production code must not include another module's private headers.
- Tests may include private headers for deliberate white-box testing.

## Primary Public Objects

A module whose main interface is a named object places that object directly under `public/` using its semantic name.

```text
include/<area>/<module>/
  public/
    <object>.hpp
    <object>.tpp  # when the object has template definitions

src/<area>/<module>/
  public/
    <object>.cpp  # when the object has out-of-line definitions
```

`<area>` is `simulation` or `core`. A primary public object is the entity, service, or application object that callers construct, hold, and operate. It owns meaningful state or behavior and is not merely a shared data representation, free implementation function, subordinate component, or subsystem manager.

Use the object's semantic name. If `XManager` would mean a separate object that manages instances of `X`, then `X` itself is a primary public object and belongs in `public/x.hpp`. A semantic header may contain a cohesive object family, such as the `Frame` hierarchy or the `UDPIn` and `UDPOut` pair; unrelated primary objects use separate semantic headers.

`frames/public/frame.hpp`, `vehicles/public/aircraft.hpp`, `qp/public/solver.hpp`, and `runner/public/runner.hpp` are the current simulation examples. Core examples include `connection/public/udp.hpp` and `devices/public/joystick.hpp`.

A primary public object may coexist with the module's applicable public operations, `data/`, `detail/`, component, policy, adapter, scheduling, or wrapper directories.

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

Public operation interfaces are first-class module capabilities, not internal implementation detail. They are not data helpers unless their purpose is representation assembly, and they are not `detail` unless they are domain calculations. Current examples include `settings/public/application.hpp`, `trim/public/application.hpp`, `trim/public/inspection.hpp`, and `io/camera/public/capture.hpp`. Core parsers, writers, and file interfaces are specialized public operations with conventional names.

A module may expose both primary public objects and public operations when both roles exist.

## Reading Layout Trees

The trees below enumerate permitted role locations, not mandatory scaffolding. Create a directory or file only when the module contains code with that role; do not create empty categories to reproduce the full tree. A `.cpp` exists only when its header has out-of-line definitions, and a `.tpp` exists only when its header has template definitions.

A module with components necessarily has at least one public component and a manager that owns or selects its components. All other displayed roles are conditional. A module without components has no component hierarchy; its data, detail, manager, and primary-object roles are present only when applicable.

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
  public/
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
      <calculation>.hpp
      <calculation>.tpp  # optional
    manager.hpp          # when the module manages one subsystem
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
    data/
    detail/
    manager.cpp          # only when a manager exists
  private/
    data/
    detail/
```

## Runner Support Roles

`Runner` is a primary public object whose behavior is top-level application orchestration. Wrappers isolate orchestration responsibilities and runner-level persistent state without representing independent subsystems. Scheduling and external-system adapters remain separate from subsystem wrappers. These support roles do not create a separate placement or naming category for `Runner` itself.

```text
include/simulation/runner/
  public/
    runner.hpp
    adapters/
      <adapter>.hpp
    data/
      types.hpp
    scheduling/
      scheduler.hpp
    wrappers/
      <wrapper>.hpp
  private/
    data/
      helpers.hpp        # optional
    detail/
      <implementation>.hpp
      <implementation>.tpp  # optional

src/simulation/runner/
  public/
    runner.cpp
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
```

Runner wrappers coordinate subsystem managers and exchange per-step data through explicit payloads and `StepContext`.

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

Intrinsic value-object behavior may remain on its owning type. Examples include representation accessors and conversions such as the methods on dynamics orientation types. Module orchestration and calculations involving independent domain objects do not belong here.

Constants owned by one domain remain in that domain's data. Constants with no domain owner that are used throughout the codebase belong in the shared `constants` module.

### Public operations

Contains cohesive free-function capabilities exposed as part of a module's public API. The semantic filename names the operation, such as `application.hpp`, `inspection.hpp`, or `capture.hpp`. Representation assembly belongs in `data/helpers`, and domain calculations belong in `detail`.

### `data/helpers`

Contains operations that assemble, pack, unpack, format, or otherwise construct data representations. This includes `make_*` functions whose purpose is to assemble an output structure.

### `detail`

Contains module calculations and implementation support that does not belong to data, components, policies, managers, adapters, scheduling, or wrappers. Detail files contain functions; they do not define shared types. Forward declarations of types owned elsewhere may appear when required by function declarations. A module's detail calculations must consume data directly and must not depend on that module's manager.

Public detail functions are domain-owned calculations required by other modules or public template definitions. A generally reusable calculation with no domain owner belongs in `util`, not in a domain module's detail directory.

Private detail functions may also provide internal validation, translation, construction, streaming, filesystem, or other implementation support required only by the owning module. A helper shared by multiple components belongs in private detail unless it is a selectable implementation strategy. Detail files use names that identify their operation; the generic `helpers.hpp` and `helpers.tpp` names are reserved for data helpers.

### `components`

Contains one file per distinct functional responsibility owned or selected by a subsystem manager. A component may own its own state and behavior. Individual formulas, repeated data entries, and intermediate stages of one calculation are not components. `base.hpp` exists only when components share behavior. `collection.hpp` exists only when a named aggregate of components is useful.

An algorithm used by only one component belongs under `private/components/<owner>/`.

### `policies`

Contains selectable or replaceable implementation strategies that satisfy a defined behavioral role. A policy may be shared by multiple components. Code does not become a policy merely because multiple components call it; shared support without strategy selection belongs in `private/detail`.

### `manager`

Represents management of one subsystem. It owns that subsystem's orchestration and, when applicable, persistent state, component selection, input-payload distribution, and output-payload aggregation. It is declared in `public/manager.hpp` and named for the subsystem it manages, such as `ControlManager` or `SensorManager`. Calculation detail should remain in components or `detail/`.

`step(input) -> output` is the standard per-tick entry point, not the manager's only permitted method. Managers may use named member methods for coherent orchestration phases; simple managers may orchestrate directly in `step()` without artificial helpers. A member method's visibility under the project's struct convention is not a reason to inline it.

An object does not become a manager merely because it coordinates other objects or owns persistent state. A named domain entity, tool, or application service is a primary public object; a class whose public role is managing subordinate objects, modes, or calculations belonging to one subsystem is a manager. A cohesive subsystem manager may have no components when splitting its behavior would create artificial responsibilities; `StructuralManager` is the current example.

### `adapters`

Contains boundaries between the application and an external system or protocol. Adapters translate application data and lifecycle operations without owning an independent simulation subsystem.

### `scheduling`

Contains application timing and decisions about when work executes. Scheduling does not implement the subsystem work being scheduled.

### `wrappers`

Contains runner-level coordination for one application phase involving subsystem managers, scheduling, and shared `StepContext` data. A wrapper is not an independent subsystem and does not replace the managers it coordinates.

## Source Mirroring

Source directories mirror the semantic category and visibility of their declarations. `<area>` is `simulation` or `core`, and `<module>` may contain multiple path segments for a nested core module:

```text
include/<area>/<module>/public/data/helpers.hpp
  -> src/<area>/<module>/public/data/helpers.cpp
include/<area>/<module>/public/data/types.hpp
  -> src/<area>/<module>/public/data/types.cpp
include/<area>/<module>/public/components/foo.hpp
  -> src/<area>/<module>/public/components/foo.cpp
include/<area>/<module>/public/adapters/foo.hpp
  -> src/<area>/<module>/public/adapters/foo.cpp
include/<area>/<module>/public/scheduling/foo.hpp
  -> src/<area>/<module>/public/scheduling/foo.cpp
include/<area>/<module>/public/wrappers/foo.hpp
  -> src/<area>/<module>/public/wrappers/foo.cpp
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
include/<area>/<module>/private/detail/foo.hpp
  -> src/<area>/<module>/private/detail/foo.cpp
include/<area>/<module>/private/policies/<policy>/foo.hpp
  -> src/<area>/<module>/private/policies/<policy>/foo.cpp
```

Template definitions remain in the corresponding `.tpp`. Every `.hpp` and `.tpp` uses `#pragma once`.

## Exceptions

- C or Fortran interoperability headers may use the extension required by the interface.

## Core Modules

Core modules follow the same `public/` and `private/` visibility rules without forcing simulation-specific manager or component roles onto infrastructure code.

- Primary public infrastructure objects follow the primary-object rule, such as `connection/public/udp.hpp` and `devices/public/joystick.hpp`.
- Shared value types and representation helpers use `public/data/types.hpp` and `public/data/helpers.hpp`.
- IO subsystems with persistent orchestration use `public/data/types.hpp` and `public/manager.hpp`.
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
- No detail calculation depends on its own manager.
- Primary public objects and public operations use semantic root-level headers. Objects that manage one subsystem use `public/manager.hpp` and the `<Subsystem>Manager` name.
- Component-specific implementations remain with their owner; selectable shared strategies use `private/policies`; other shared private support uses `private/detail`.
- Source paths mirror declaration visibility and semantic role.
- Core parser, parsing, validation, writer, and file interfaces use their defined semantic paths.
- No obsolete flat `public.hpp`, `private.hpp`, or legacy submodule directories remain after a module is refactored.
- Do not add nested namespaces.
