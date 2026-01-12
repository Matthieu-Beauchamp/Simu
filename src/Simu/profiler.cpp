////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2026 Matthieu Beauchamp-Boulay
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

#include "profiler.hpp"

#if defined(SIMU_ENABLE_TRACY) || defined(SIMU_CUSTOM_PROFILER) || true

#    include <cstdlib>
#    include <new>


// See https://en.cppreference.com/w/cpp/memory/new/operator_new.html

// no inline, required by [replacement.functions]/3
void* operator new(std::size_t sz) {
    if (sz == 0)
        ++sz; // avoid std::malloc(0) which may return nullptr on success

    if (void* ptr = std::malloc(sz)) {
        SIMU_PROFILE_ALLOC(ptr, sz);
        return ptr;
    }

    throw std::bad_alloc{}; // required by [new.delete.single]/3
}

// no inline, required by [replacement.functions]/3
void* operator new[](std::size_t sz) {
    if (sz == 0)
        ++sz; // avoid std::malloc(0) which may return nullptr on success

    if (void* ptr = std::malloc(sz)) {
        SIMU_PROFILE_ALLOC(ptr, sz);
        return ptr;
    }

    throw std::bad_alloc{}; // required by [new.delete.single]/3
}

void operator delete(void* ptr) noexcept {
    SIMU_PROFILE_FREE(ptr, size);
    std::free(ptr);
}

void operator delete(void* ptr, [[maybe_unused]] std::size_t size) noexcept {
    SIMU_PROFILE_FREE(ptr, size);
    std::free(ptr);
}

void operator delete[](void* ptr) noexcept {
    TracyFree(ptr);
    std::free(ptr);
}

void operator delete[](void* ptr, [[maybe_unused]] std::size_t size) noexcept {
    TracyFree(ptr);
    std::free(ptr);
}

#endif