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

#include <cstdint>

#include <exception>
#include <source_location>
#include <sstream>

////////////////////////////////////////////////////////////
/// \brief Define DLL export/import macro
////////////////////////////////////////////////////////////
#define SIMU_API

// Note that these macros are defined by cmake (cmake/config.cmake),
// this is because compiler specific macros are not very reliable,
// msys/gcc would define Linux as the platform on Windows.
#if defined(SIMU_OS_WINDOWS)
#    undef SIMU_API
#    if defined(SIMU_EXPORT)
#        define SIMU_API __declspec(dllexport)
#    else
#        define SIMU_API __declspec(dllimport)
#    endif

#elif defined(SIMU_OS_LINUX)

#elif defined(SIMU_OS_MACOSX)

#else
#    error "Unrecognized OS"
#endif


#define SIMU_ASSERT(cond, msg)                                                 \
    if (!(cond))                                                               \
    throw simu::Exception(msg)


namespace simu
{

class Exception : public std::exception
{
public:

    Exception(
        const char*          msg,
        std::source_location loc = std::source_location::current()
    )
        : std::exception{}
    {
        std::stringstream ss{};
        ss << "From " << loc.function_name() << " in " << loc.file_name() << ":"
           << loc.line() << '\n'
           << msg;
        msg_ = ss.str();
    }

    char const* what() const override { return msg_.c_str(); }

private:

    std::string msg_;
};

typedef std::int8_t  Int8;
typedef std::uint8_t Uint8;

typedef std::int16_t  Int16;
typedef std::uint16_t Uint16;

typedef std::int32_t  Int32;
typedef std::uint32_t Uint32;

typedef std::int64_t  Int64;
typedef std::uint64_t Uint64;

} // namespace simu
