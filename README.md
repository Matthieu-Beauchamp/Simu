# Simu
Simu is a 2D physics engine aimed towards real-time simulation of rigid bodies and
gravitationnal/magnetic fields.

Simu includes a graphics modules to facilitate visualization of the simulations.


# Installation

## Supported compilers:
Simu expects a C++20 compliant compiler. The builds are tested against the following compilers:
- MSVC 17 2022 (17.5.4)
- GCC 12.1
- 


## Building with CMake
System-wide installation is not yet supported, the recommended installation is 
to build Simu within your project, either with CMake's fetch content or as 
a git submodule.

In both cases, the following libraries can be linked against:
- `simu-math`
- `simu-physics` (implies `simu-math`)
- `simu-graphics` (implies `simu-physics`)

Your target should have the proper include directories simply by linking to Simu's
libraries. Includes use the following pattern: `#include "Simu/math/maths.hpp"`
and only the main library headers should be included.
