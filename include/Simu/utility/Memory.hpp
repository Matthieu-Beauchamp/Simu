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

// TODO: Could keep track of the biggest available slot to avoid searches when
//  size + (alignement-1) > slotSize
template <std::size_t size>
class Block
{
    Block();

    template <class T>
    T* allocate(std::size_t n);

    template <class T>
    bool deallocate(T* p, std::size_t n) noexcept;


private:

    typedef std::uint8_t Byte;

    struct FreeBlock
    {
        std::size_t size() const noexcept { return end - start; }

        Byte* start;
        Byte* end;
    };

    std::list<FreeBlock> freeList_{};

    Byte[size] data_;
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


template <std::size_t sz>
Block<sz>::Block()
{
    freeList_.emplace_back(data_, data_ + sz);
}

template <std::size_t sz>
template <class T>
T* Block<sz>::allocate(std::size_t n)
{
    if (n == 0)
        return nullptr;

    std::size_t minSize = n * sizeof(T);

    for (auto it = freeList_.begin(); it != freeList_.end(); ++it)
    {
        void*       alignedStart = it->start;
        std::size_t blockSize    = it->size();

        if (std::align(alignof(T), minSize, alignedStart, blockSize) != nullptr)
        {
            if (alignedStart != it->start)
                freeList_.emplace(it, it->start, alignedStart);

            Byte* alignedEnd = static_cast<Byte*>(alignedStart) + minSize;
            if (alignedEnd != it->end)
                it->start = alignedEnd;
            else
                freeList_.erase(it);

            return static_cast<T*>(alignedStart);
        }
    }

    return nullptr;
}

template <std::size_t sz>
template <class T>
bool Block<sz>::deallocate(T* p, std::size_t n) noexcept
{
    if (p < data_ || p >= data_ + sz)
        return false;

    std::size_t size = n * sizeof(T);
    Byte*       end  = p + size;

    auto next = freeList_.begin();
    for (; next != freeList_.end(); ++next)
    {
        if (p < next->start)
            break;
    }

    bool canMergeWithNext = (next != freeList_.end() && end == next->start);
    bool canMergeWithPrev
        = (next != freeList_.begin() && std::prev(next)->end == p);

    if (canMergeWithNext && canMergeWithPrev)
    {
        next->start = std::prev(next)->start;
        freeList_.erase(prev);
    }
    else if (canMergeWithNext)
        next->start = p;
    else if (canMergeWithPrev)
        std::prev(next)->end = end;
    else
        freeList_.emplace(next, p, end);

    return true;
}


} // namespace simu
