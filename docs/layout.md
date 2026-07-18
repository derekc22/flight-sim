# C++ Layout Rules

## Glossary

- **Symbol:** A named C++ item, such as a function, type, variable, alias, or constant.
- **Owner:** The smallest module or submodule responsible for a symbol.
- **Public API:** Declarations in `public.*` that code outside the owner may use.
- **Public interface type:** A type named in a public declaration, such as a parameter, return type, field, or alias. Callers need its declaration to compile.
- **Implementation detail:** A symbol used only inside `.cpp` or private files. Callers do not need its declaration.
- **Shared public type:** A public interface type named by multiple sibling public APIs that has no natural single sibling owner.
- **Aggregate header:** A parent `public.hpp` that collects child public headers for convenient inclusion.

Rules for active C++ code under these roots:

```text
include/simulation
src/simulation
include/core
src/core
```

## Core Rule

Public symbols belong in `public.*`.

Private symbols belong in `private.*`.

The owner of a symbol is the module or submodule directory where the symbol is
declared. For example, a symbol declared in
`include/simulation/transforms/so3/public.hpp` is owned by
`simulation/transforms/so3`. A symbol declared in
`include/core/json/control/private.hpp` is owned by `core/json/control`.

Sibling submodules are submodule directories with the same immediate parent.
For example, `simulation/transforms/so3`, `simulation/transforms/s3`, and
`simulation/transforms/se3` are sibling submodules under
`simulation/transforms`. `core/json/control`, `core/json/estimation`, and
`core/json/guidance` are sibling submodules under `core/json`.

A symbol is public when any of these files includes it, calls it, constructs it,
stores it, names it, or needs it to compile a declaration:

- A file in a different module.
- For `simulation` symbols, any file in `src/core`, JSON, messages, or IO.
- A public header or public `.tpp` in any sibling submodule.

`src/main.cpp` and tests are one-way consumers. Their private uses do not force
symbols into `public.*`.

A symbol is private when all uses stay in one of these places:

- The owner.
- A sibling submodule under the same immediate parent, limited to that sibling's
  `.cpp`, `private.hpp`, or `private.tpp` files.
- The body of a non-template function or member function defined in the owner's
  `.cpp` files.

## File Roles

These are allowed file roles, not required files:

```text
include/<area>/<module>/public.hpp
include/<area>/<module>/public.tpp
include/<area>/<module>/private.hpp
include/<area>/<module>/private.tpp
src/<area>/<module>/public.cpp
src/<area>/<module>/private.cpp
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
If a module has no public declarations, no private declarations, and no source
definitions, it should have no files.

Do not keep unnecessary empty files. Empty namespace blocks, include-only
wrappers, and blank source/template files are invalid unless an active source or
header includes them.

## Symbol Placement

Put a symbol in `public.*` when a file outside the owner uses it, except for
uses from sibling `.cpp`, `private.hpp`, or `private.tpp` files under the same
immediate parent. `src/main.cpp` and tests do not count as outside the owner
for this rule. For `simulation` symbols, `src/core`, JSON, messages, and IO
always count as outside the owner.

Also put a symbol in `public.*` when any of these are true:

- It appears in a public function signature.
- It appears in a public type field.
- It appears in a public alias.
- It appears in a public template parameter or return type.
- It is a member function of a public struct or class.
- A public template definition in `public.tpp` calls it.

Put a symbol in `private.*` only when all uses stay in the owner or in sibling
`.cpp`, `private.hpp`, or `private.tpp` files under the same immediate parent. A
symbol cannot be private when a public header or public `.tpp` outside the owner
names it, or when it appears in a public signature or public type layout.

Member functions adopt the placement status of their owning type. A member
function of a public struct or class is a public symbol and belongs in
`public.*`. A member function of a private struct or class is a private symbol
and belongs in `private.*`.

Public template helper rule:

If a helper appears in the body of a public template definition in `public.tpp`,
keep that helper in `public.*`. This is because the C++ preprocessor literally pastes the contents of public.tpp into any translation unit that includes public.hpp.
Thus, because public.tpp contents are exposed through public.hpp, this repo requires helpers called by public.tpp to live in public.*.

```cpp
// include/simulation/foo/public.hpp
namespace foo {
    template <typename T>
    T public_func(T x);
}

#include "simulation/foo/public.tpp"
```

```cpp
// include/simulation/foo/public.tpp
namespace foo {
    template <typename T>
    T public_func(T x) {
        return private_helper(x);
    }
}
```

This is invalid under these layout rules. `private_helper` is needed by a public
template body, so `private_helper` must be moved to `public.*` or the public
template must be redesigned so it does not call a private helper.

Helper visibility relations:

This list is about direct references to private symbols. For `.hpp`, this means
a declaration names a private type, alias, constant, or function in a signature,
alias, or type layout. For `.cpp` and `.tpp`, this usually means code calls a
private helper function.

- Owner `public.hpp -> private symbol`: disallowed.
- Owner `public.tpp -> private helper function`: disallowed.
- Owner `public.cpp -> private helper function`: allowed.
- Owner `private.hpp -> private symbol`: allowed.
- Owner `private.tpp -> private helper function`: allowed.
- Owner `private.cpp -> private helper function`: allowed.
- Sibling `.cpp`, `private.hpp`, or `private.tpp -> private symbol`: allowed
  only under the same immediate parent.
- Sibling `public.hpp` or `public.tpp -> private symbol`: disallowed.
- Different module file `-> private symbol`: disallowed, except for the
  `src/main.cpp` and test one-way-consumer exception.

Who may include private headers:

Here `A -> B` means `A` includes `B`.

- `private.hpp` or `private.tpp -> public.hpp`: allowed when the private code
  uses the public API.
- `public.hpp -> private.hpp`: disallowed.
- `public.tpp -> private.hpp`: disallowed.

Which `.tpp` file belongs with which header:

Here `A -> B` means `A` includes `B`.

- `public.hpp -> public.tpp`: allowed for public template definitions.
- `private.hpp -> private.tpp`: allowed for private template definitions.
- `public.hpp -> private.tpp`: disallowed.
- `public.tpp -> private.tpp`: disallowed.
- `private.tpp -> public.tpp`: allowed through `public.hpp` when the private
  template uses a public template definition.

A helper used only by non-template definitions in the owner's `public.cpp` may
be private when it is not in a public signature or data layout. The same rule
applies to helpers used only by public member function definitions in the
owner's `public.cpp`. This helper exception applies to free functions, not to
member functions of public types.

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

Use subfolders for module boundaries such as `actuators/propulsor`,
`actuators/surface`, `avionics/sensors`, `avionics/computers`,
`control/shared`, `control/pid/controllers/<controller>`,
`control/linear_quadratic/controllers/<controller>`,
`estimation/kalman/estimators/<estimator>`, `transforms/s3`,
`transforms/se3`, `transforms/so3`, and `util/<utility>`.

Core modules use the same layout. Top-level core modules include
`connection`, `interface`, `io`, `json`, and `messages`. Core JSON parser
submodules must be real subfolders, such as:

```text
include/core/json/control/public.hpp
include/core/json/control/private.hpp
src/core/json/control/public.cpp
src/core/json/control/private.cpp
```

Do not keep independent submodules as flat files like `controllers/lqr.hpp` or
`transforms/so3.hpp`. Do not keep flat core paths such as
`core/json/control.hpp`, `core/json/json.hpp`, `core/io/io.hpp`,
`core/connection/connection.hpp`, or `core/messages/messages.hpp`.

## Includes And Aggregates

Headers must include what they directly use. Do not rely on accidental
transitive includes.

- Public headers may include `public.hpp` headers for symbols they directly
  name.
- Public headers must not include another module's `private.hpp`.
- Private headers may include their owner's `public.hpp` and public headers for
  symbols they directly name.
- Source files may include their owner's `private.hpp`.
- Source files, `private.hpp`, and `private.tpp` may include a sibling
  `private.hpp` under the same immediate parent.
- Public headers and public `.tpp` files must not include any `private.hpp`
  outside their owner.
- In headers, place `.hpp` includes at the top after `#pragma once` and before
  declarations or definitions.
- The matching `.tpp` include is the only include that may appear at the bottom
  of a header.
- Source files must include their owning header directly.
- Standard library headers must be included directly by the file that uses them.
- Do not use broad aggregate headers to hide missing direct includes.

Module files outside the owner should include public headers, for example
`simulation/actuators/propulsor/public.hpp`, not private owner headers such as
`simulation/actuators/propulsor/private.hpp`. Core files follow the same rule:
use `core/json/control/public.hpp`, not `core/json/control/private.hpp`, from
outside `core/json/control`.

`src/main.cpp` and tests may include private headers for executable wiring and
white-box coverage.

A module-level `public.hpp` can be a used public aggregate. A used aggregate may
contain no declarations after its includes; an unused aggregate wrapper should
be deleted. Do not add forwarding wrappers for deleted flat paths unless the
user explicitly asks for compatibility wrappers.

## Current Exceptions

New exceptions require explicit user approval.

### Core Interface Placeholder

These files are intentionally retained as empty placeholders:

```text
include/core/interface/public.hpp
src/core/interface/public.cpp
```

Reason: `core/interface` is reserved as a module boundary by explicit user
request, even though it currently has no declarations or definitions.

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

1. Identify every symbol used outside its owner and outside sibling `.cpp`,
   `private.hpp`, or `private.tpp` files under the same immediate parent, then
   put it in `public.*`. Ignore uses from `src/main.cpp` and tests.
2. Put private-only symbols in `private.*`.
3. Put template definitions in the matching `.tpp`.
4. Put non-template definitions in the matching `.cpp`.
5. Include direct dependencies, not broad aggregates.
6. Make each source file include its owning header directly.
7. Use a real subfolder for independent submodules.
8. Verify no public header includes another module's private header.
9. Verify no `.hpp` include appears after declarations or definitions.
10. Verify no deleted old path, flat core path, or flat `shared.*` path is
    referenced.
11. Verify no unnecessary empty shell files were created.
12. Verify private free functions are declared in `private.hpp` and defined in
    `private.cpp`.
13. Smoke compile the new public and private headers independently.

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
if a file outside `foo` must compile the `foo::public_function()` declaration.

### Public Class Member

Member functions of public types are public symbols. Declare public type member
functions in `public.hpp`, and define public type member functions in
`public.cpp` or `public.tpp`.

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

        int private_member(int value);
    };

    PrivateType make_private_type(int value);
}
```

Member functions of private types are private symbols. Declare private type
member functions in `private.hpp`, and define private type member functions in
`private.cpp` or `private.tpp`.

Move it to `public.hpp` only if a file outside the owner must name it or it
appears in a public signature/data layout.

### Public Template

Templates instantiated by files outside the owner must be public.

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

If sibling submodules both need a helper, choose based on which files name it:

```cpp
// include/simulation/parent/child_a/private.hpp
namespace parent {
    int child_a_helper(int value);
}
```

Sibling submodules under the same immediate parent may include each other's
`private.hpp` from `.cpp`, `private.hpp`, or `private.tpp` files. Use public
placement only if the helper is declared by the parent public header, a file
outside the parent names it, or it appears in a public signature/data layout. Do
not create an ad hoc `shared.*` file for sibling implementation sharing.

### Public Interface Types Shared Across Sibling Submodules

Use a real `shared` submodule for a public interface type that is named in
multiple sibling public headers or signatures and has no natural single sibling
owner.

“Exposed or named” means it appears in a public header, so callers need its
declaration:

```cpp
// include/simulation/transforms/so3/public.hpp
Eigen::Matrix3d eul_to_R(double a, double b, double c, EulerOrder order);
```

`EulerOrder` is named in the public API.

“Called” means a function is only used inside a `.cpp` implementation:

```cpp
// src/simulation/transforms/so3/public.cpp
return normalize_and_canonicalize(q);
```

Users of SO3 never need to know that `normalize_and_canonicalize()` was called
internally. A function merely called by a sibling implementation remains owned
by its natural submodule; that call alone does not make the function shared.

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

### Shared Control Submodule

Use a real `control/shared` submodule for shared control interface types.

```cpp
// include/simulation/control/shared/public.hpp
#pragma once
#include "simulation/dynamics/public.hpp"

namespace control {
    struct ControlOutput;
    struct ControllerInput;
}
```

Control child public headers may include it for shared control interface types
without including the parent `simulation/control/public.hpp` aggregate. Do not
create flat `shared.hpp`, `shared.tpp`, or `shared.cpp` files.
