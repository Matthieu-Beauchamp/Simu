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

#include <cstddef>
#include <memory>
#include <list>

namespace simu
{

template <std::size_t size>
class Block
{
    Block() noexcept = default;

    bool isFull() const noexcept;

    template <class T>
    T* allocate(std::size_t n);

    template <class T>
    void deallocate(T* p, std::size_t n) noexcept;


private:

    std::uint8_t[blockSize] data;
};


// https://github.com/mtrebi/memory-allocators
// https://en.cppreference.com/w/cpp/named_req/Allocator
template <class T, std::size_t blockSize>
class FreeListAllocator
{
public:

    typedef T           value_type;
    typedef value_type* pointer;
    typedef std::size_t size_type;

    template <class U>
    struct rebind
    {
        typedef FreeListAllocator<U, blockSize> other;
    };

    FreeListAllocator();

    template <class U>
    FreeListAllocator(const FreeListAllocator<U, blockSize>& other) noexcept;

    template <class U>
    FreeListAllocator&
    operator=(const FreeListAllocator<U, blockSize>& other) noexcept;

    template <class U>
    FreeListAllocator(FreeListAllocator<U, blockSize>&& other) noexcept;

    template <class U>
    FreeListAllocator&
    operator=(FreeListAllocator<U, blockSize>&& other) noexcept;


    pointer allocate(size_type n);
    void    deallocate(pointer p, size_type n) noexcept;

    constexpr size_type max_size() const noexcept;


    template <class U>
    bool operator==(const FreeListAllocator<U, blockSize>& other) const noexcept;

    template <class U>
    bool operator!=(const FreeListAllocator<U, blockSize>& other) const noexcept;

private:

    std::shared_ptr<std::list<Block<blockSize>>> blocks_;
};


} // namespace simu
