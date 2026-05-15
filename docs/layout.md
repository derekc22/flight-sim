# Simulation Layout Rules

Rules for active `include/simulation` and `src/simulation` code.

## Core Rule

Publicly used symbols belong in `public.*`.

Module-private symbols belong in `private.*`.

A symbol is public if code outside its owning module or submodule includes it,
calls it, constructs it, stores it, names it, or needs it to compile a public
interface. A symbol is private only if it is used entirely inside its owning
module or submodule.

## File Roles

These are allowed file roles, not required files:

```text
include/simulation/<module>/public.hpp
include/simulation/<module>/public.tpp
include/simulation/<module>/private.hpp
include/simulation/<module>/private.tpp
src/simulation/<module>/public.cpp
src/simulation/<module>/private.cpp
```

- `public.hpp`: public declarations, types, aliases, constants, and template declarations.
- `public.tpp`: public template definitions declared in `public.hpp`.
- `private.hpp`: private declarations, helper types, aliases, constants, and template declarations.
- `private.tpp`: private template definitions declared in `private.hpp`.
- `public.cpp`: non-template definitions declared in `public.hpp`.
- `private.cpp`: non-template definitions declared in `private.hpp`.

Create only files that own real declarations or definitions. Omit `public.cpp`
when there are no public non-template definitions. Omit `private.hpp` when
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

Put a symbol in `private.*` only when the owning module/submodule is the only
user, no outside public header needs it, no external code names it, and it does
not appear in a public signature/data layout.

Public declarations belong in `public.hpp`; public template definitions in
`public.tpp`; public non-template definitions in `public.cpp`.

Private declarations belong in `private.hpp`; private template definitions in
`private.tpp`; private non-template definitions in `private.cpp`.

Private free-function helpers belong in `private.hpp` and `private.cpp`.

Private helpers may stay in the module namespace. File placement, not namespace
nesting, determines public vs private. Do not add nested namespaces.

## Submodules

Multi-module areas must use real subfolders. A submodule follows the same
`public.*` and `private.*` rules as a top-level module.

Example:

```text
include/simulation/actuators/propulsor/public.hpp
include/simulation/actuators/propulsor/private.hpp
src/simulation/actuators/propulsor/public.cpp
src/simulation/actuators/propulsor/private.cpp
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
- Public headers must not include another module's `private.hpp`.
- Private headers may include their owning `public.hpp` and direct public dependencies.
- Source files must include their owning header directly.
- Standard library headers must be included directly by the file that uses them.
- Do not use broad aggregate headers to hide missing direct includes.

External code should include public headers, for example
`simulation/actuators/propulsor/public.hpp`, not private owner headers such as
`simulation/actuators/propulsor/private.hpp`.

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
2. Put private-only symbols in `private.*`.
3. Put template definitions in the matching `.tpp`.
4. Put non-template definitions in the matching `.cpp`.
5. Include direct dependencies, not broad aggregates.
6. Make each source file include its owning header directly.
7. Use a real subfolder for independent submodules.
8. Verify no public header includes another module's private header.
9. Verify no deleted old path or non-control `shared.*` path is referenced.
10. Verify no unnecessary empty shell files were created.
11. Verify private free functions are declared in `private.hpp` and defined in
    `private.cpp`.
12. Smoke compile the new public and private headers independently.

## Refactor Constraints

During structural refactors:

- Preserve existing namespaces, symbols, signatures, definitions, behavior, and
  comments unless the user explicitly approves a change.
- Do not touch CARE/SLICOT files unless explicitly asked.

## Examples

### Public Function With Private Helper

A helper used only to implement a public function is private. Publicness is
determined by who can name or use the helper directly, not by whether a public
function calls it internally.

```cpp
// include/simulation/foo/public.hpp
namespace foo {
    int public_function(int value);
}
```

```cpp
// include/simulation/foo/private.hpp
namespace foo {
    int private_helper(int value);
}
```

```cpp
// src/simulation/foo/public.cpp
#include "simulation/foo/public.hpp"
#include "simulation/foo/private.hpp"

namespace foo {
    int public_function(int value) {
        return private_helper(value);
    }
}
```

```cpp
// src/simulation/foo/private.cpp
#include "simulation/foo/private.hpp"

namespace foo {
    int private_helper(int value) {
        return value;
    }
}
```

`foo::private_helper()` stays private if only `foo::public_function()` or other `foo`
implementation files call it.

### Private Helper Used By One Function

If a helper is module-private, put its declaration in `private.hpp` and its
definition in `private.cpp`, even when only one public function calls it.

```cpp
// include/simulation/foo/private.hpp
namespace foo {
    int private_helper(int value);
}
```

```cpp
// src/simulation/foo/private.cpp
#include "simulation/foo/private.hpp"

namespace foo {
    int private_helper(int value) {
        return value;
    }
}
```

Do not hide this helper inside the `.cpp`; declare it in `private.hpp`.

### Helper Used By Another Module

If another module calls, names, constructs, or stores the helper, it is not
private. Move it to the owning module's `public.*`.

```cpp
// include/simulation/foo/public.hpp
namespace foo {
    int reusable_function(int value);
}
```

```cpp
// src/simulation/bar/public.cpp
#include "simulation/foo/public.hpp"

namespace bar {
    int public_function(int value) {
        return foo::reusable_function(value);
    }
}
```

Do not make `bar` include `simulation/foo/private.hpp`.

### Symbol Appears In A Public Signature

If a type appears in a public function signature, public struct field, public
alias, or public template parameter, that type must be public too.

```cpp
// include/simulation/foo/public.hpp
namespace foo {
    struct PublicArgument {
        int value = 0;
    };

    struct PublicResult {
        int value = 0;
    };

    PublicResult public_function(PublicArgument argument);
}
```

`foo::PublicArgument` and `foo::PublicResult` cannot live only in `private.hpp`
if external code must compile the `foo::public_function()` declaration.

### Public Class Member

Public member functions of public types are public symbols.

```cpp
// include/simulation/foo/public.hpp
namespace foo {
    struct PublicType {
        int public_member(int value);
    };
}
```

```cpp
// src/simulation/foo/public.cpp
#include "simulation/foo/public.hpp"

namespace foo {
    int PublicType::public_member(int value) {
        return value;
    }
}
```

Do not define public members in `private.cpp`.

### Private Class Or Struct

A type used only to organize implementation details stays private.

```cpp
// include/simulation/foo/private.hpp
namespace foo {
    struct PrivateType {
        int value = 0;
    };

    PrivateType make_private_type(int value);
}
```

Move it to `public.hpp` only if external code must name it or it appears in a
public signature/data layout.

### Public Template

Templates that external code instantiates must be public.

```cpp
// include/simulation/foo/public.hpp
namespace foo {
    template <typename T>
    T public_template(T value);
}

#include "simulation/foo/public.tpp"
```

```cpp
// include/simulation/foo/public.tpp
namespace foo {
    template <typename T>
    T public_template(T value) {
        return value;
    }
}
```

`public.hpp` should include `public.tpp` when callers need the template
definition.

### Private Template

Templates used only by module implementation are private.

```cpp
// include/simulation/foo/private.hpp
namespace foo {
    template <typename T>
    T private_template(T value);
}

#include "simulation/foo/private.tpp"
```

```cpp
// include/simulation/foo/private.tpp
namespace foo {
    template <typename T>
    T private_template(T value) {
        return value;
    }
}
```

### Public Source Calling Private Template

A public `.cpp` may include its module's own `private.hpp` when a public
definition needs private implementation helpers.

```cpp
// src/simulation/foo/public.cpp
#include "simulation/foo/public.hpp"
#include "simulation/foo/private.hpp"

namespace foo {
    int public_function(int value) {
        return private_template(value);
    }
}
```

This is allowed because `public.cpp` is still inside the owning module. Public
headers should not include another module's `private.hpp`.

### Private Source Calling Public API

A private implementation file may include its own `public.hpp` and any direct
public dependencies it uses.

```cpp
// src/simulation/foo/private.cpp
#include "simulation/foo/private.hpp"
#include "simulation/foo/public.hpp"
#include "simulation/bar/public.hpp"

namespace foo {
    int private_helper(int value) {
        return bar::public_function(value);
    }
}
```

Do not include `bar/private.hpp`.

### Submodule Private Helper

Private symbols belong to the smallest owning submodule that uses them.

```cpp
// include/simulation/parent/child/private.hpp
namespace parent {
    int child_private_helper(int value);
}
```

A child-only helper belongs in `parent/child/private.*`, not in
`parent/private.*` or `parent/public.hpp`.

### Helper Shared Across Sibling Submodules

If sibling submodules both need a helper, choose based on visibility:

```cpp
// include/simulation/parent/public.hpp
#include "simulation/parent/child_a/public.hpp"
#include "simulation/parent/child_b/public.hpp"

namespace parent {
    int shared_public_function(int value);
}
```

Use public placement if the helper is part of the cross-submodule API. If it is
only an implementation detail shared by sibling internals, stop and decide on an
explicit owning module/submodule instead of creating an ad hoc `shared.*` file.

### Aggregates

A used aggregate can be declaration-free after its includes.

```cpp
// include/simulation/parent/public.hpp
#pragma once
#include "simulation/parent/child_a/public.hpp"
#include "simulation/parent/child_b/public.hpp"
```

This is not an empty shell if active code includes it as the public aggregate.
Delete unused aggregate wrappers.

### Empty Files

Do not keep files that own no declarations or definitions.

```cpp
// include/simulation/foo/private.hpp
#pragma once

namespace foo {
}
```

```cpp
// src/simulation/foo/private.cpp
#include "simulation/foo/private.hpp"
```

Create `private.*` only when there are real private symbols to own.

### Control Shared Exception

Control is the only approved shared-header exception.

```cpp
// include/simulation/control/shared.hpp
#pragma once
#include "simulation/dynamics/public.hpp"

namespace control {
    struct ControlOutput;
    struct ControllerInput;
}
```

Control child public headers may include it for shared control interface types.
Do not create other `shared.hpp`, `shared.tpp`, or `shared.cpp` files.
