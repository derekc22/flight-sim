# FPV Simulator

A drone FPV simulator built in C++.

## Building & Running

This project uses **CMake** as its build system.

### Prerequisites

- CMake 3.16+
- C++17 compatible compiler (Clang, GCC, MSVC)

### Build Commands

| Step | Command | What it does |
|------|---------|--------------|
| **1. Configure** | `cmake -B build -S .` | Reads CMakeLists.txt, generates build files |
| **2. Build** | `cmake --build build` | Compiles all `.cpp` files into executable |
| **3. Run** | `./build/fpv-sim` | Runs your program |

### Day-to-Day Workflow

After the initial configure, you only need:

```bash
cmake -B build -S . &&cmake --build build && ./build/fpv-sim
```

Re-run `cmake -B build -S .` only when you:
- Add/remove `.cpp` files
- Change `CMakeLists.txt`

**Full command**:
```bash
cmake -B build -S . &&cmake --build build && ./build/fpv-sim
```

## Project Structure

```
fpv-sim/
├── include/          # Header files (.hpp)
│   ├── core/
│   ├── display/
│   ├── interface/
│   ├── manager/
│   └── simulation/
├── src/              # Source files (.cpp)
│   ├── main.cpp
│   ├── core/
│   ├── display/
│   ├── interface/
│   ├── manager/
│   └── simulation/
├── build/            # Compiled output (git-ignored)
└── CMakeLists.txt    # Build configuration
```

### Adding New Modules

1. Create header: `include/<category>/<module>/<module>.hpp`
2. Create source: `src/<category>/<module>/<module>.cpp`
3. Reconfigure: `cmake -S . -B build`
4. Rebuild: `cmake --build build`
