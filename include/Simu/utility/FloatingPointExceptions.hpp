////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2023 Matthieu Beauchamp-Boulay
//
// This software is provided 'as-is', without any express or implied warranty.
// In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it freely,
// subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented;
//    you must not claim that you wrote the original software.
//    If you use this software in a product, an acknowledgment
//    in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such,
//    and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
//
////////////////////////////////////////////////////////////

#pragma once

#include <float.h>
#include <fenv.h>

namespace simu
{


// https://stackoverflow.com/a/4455194
// https://stackoverflow.com/a/2769889

bool isNan(float val) { return val != val; }

// void enableExceptions() { feenableexcept(FE_INVALID | FE_OVERFLOW | FE_DIVBYZERO); }

#ifdef SIMU_WINDOWS

void enableFpExceptions()
{
    unsigned int fp_control_word;
    unsigned int new_fp_control_word;


    _controlfp_s(&fp_control_word, 0, 0);

    // setting the mask disables the exception
    new_fp_control_word = fp_control_word | _EM_INVALID | _EM_DENORMAL
                          | _EM_ZERODIVIDE | _EM_OVERFLOW | _EM_UNDERFLOW
                          | _EM_INEXACT;

    // clearing enables it
    new_fp_control_word &= ~_EM_ZERODIVIDE;
    new_fp_control_word &= ~_EM_INVALID;

    _controlfp_s(&fp_control_word, new_fp_control_word, _MCW_EM);
}

#endif

} // namespace simu
