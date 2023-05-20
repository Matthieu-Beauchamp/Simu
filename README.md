[![License](https://img.shields.io/github/license/Matthieu-Beauchamp/Simu)](https://github.com/Matthieu-Beauchamp/Simu/blob/master/LICENSE)

[![Build](https://github.com/Matthieu-Beauchamp/Simu/actions/workflows/build.yml/badge.svg)](https://github.com/Matthieu-Beauchamp/Simu/actions/workflows/build.yml)

[![codecov](https://codecov.io/gh/Matthieu-Beauchamp/Simu/branch/master/graph/badge.svg?token=AYAKQV1UZI)](https://codecov.io/gh/Matthieu-Beauchamp/Simu)

[![Codacy Badge](https://app.codacy.com/project/badge/Grade/6eae28fc833d47f8858fc4fbe3328dff)](https://app.codacy.com/gh/Matthieu-Beauchamp/Simu/dashboard?utm_source=gh&utm_medium=referral&utm_content=&utm_campaign=Badge_grade)


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
libraries. Includes use the following pattern: `#include "Simu/math.hpp"`
and only the main library headers should be included.
