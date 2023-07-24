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

#include "Simu/config.hpp"

namespace simu
{

// TODO: Could keep track of the biggest available slot to avoid searches when
//  size + (alignement-1) > slotSize
template <std::size_t size>
class Block
{
public:

    Block();

    ~Block()
    {
        SIMU_ASSERT(
            freeList_.front().start == data_
                && freeList_.front().end == data_ + size,
            "Missing deallocations"
        );
    }

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

    Byte data_[size];
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

    // otherwise Deleters of derived virtual types cannot be assigned
    //  when moving unique pointers...
    typedef std::function<void(void*)> Deleter;

    // unique pointer may outlive this allocator.
    // U may not be an array type (U[]).
    template <class U, class... Args>
    std::unique_ptr<U, Deleter> makeUnique(Args&&... args)
    {
        typename rebind<U>::other alloc{*this};

        Deleter d = [=](void* obj) mutable {
            U* typed = static_cast<U*>(obj);
            typed->~U();
            alloc.deallocate(typed, 1);
        };

        U* obj = alloc.allocate(1);
        std::construct_at(obj, std::forward<Args>(args)...);
        return {obj, d};
    }


    pointer allocate(size_type n);
    void    deallocate(pointer p, size_type n) noexcept;

    constexpr size_type max_size() const noexcept;


    template <class U>
    bool operator==(const FreeListAllocator<U, blockSize>& other) const noexcept;

    template <class U>
    bool operator!=(const FreeListAllocator<U, blockSize>& other) const noexcept;

private:

    template <class U, std::size_t s>
    friend class FreeListAllocator;

    std::shared_ptr<std::list<Block<blockSize>>> blocks_;
};


////////////////////////////////////////////////////////////
// Block
////////////////////////////////////////////////////////////

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
                freeList_.emplace(it, it->start, static_cast<Byte*>(alignedStart));

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
    Byte* start = reinterpret_cast<Byte*>(p);
    if (start < data_ || start >= data_ + sz)
        return false;

    std::size_t size = n * sizeof(T);
    Byte*       end  = start + size;

    auto next = freeList_.begin();
    for (; next != freeList_.end(); ++next)
    {
        if (start < next->start)
            break;
    }

    bool canMergeWithNext = (next != freeList_.end() && end == next->start);
    bool canMergeWithPrev
        = (next != freeList_.begin() && std::prev(next)->end == start);

    if (canMergeWithNext && canMergeWithPrev)
    {
        auto prev   = std::prev(next);
        next->start = prev->start;
        freeList_.erase(prev);
    }
    else if (canMergeWithNext)
        next->start = start;
    else if (canMergeWithPrev)
        std::prev(next)->end = end;
    else
        freeList_.emplace(next, start, end);

    return true;
}


////////////////////////////////////////////////////////////
// FreeListAllocator
////////////////////////////////////////////////////////////

template <class T, std::size_t sz>
FreeListAllocator<T, sz>::FreeListAllocator()
    : blocks_{std::make_shared<std::list<Block<sz>>>()}
{
}

template <class T, std::size_t sz>
template <class U>
FreeListAllocator<T, sz>::FreeListAllocator(const FreeListAllocator<U, sz>& other
) noexcept
{
    *this = other;
}

template <class T, std::size_t sz>
template <class U>
FreeListAllocator<T, sz>&
FreeListAllocator<T, sz>::operator=(const FreeListAllocator<U, sz>& other) noexcept
{
    blocks_ = other.blocks_;
    return *this;
}

template <class T, std::size_t sz>
template <class U>
FreeListAllocator<T, sz>::FreeListAllocator(FreeListAllocator<U, sz>&& other
) noexcept
{
    *this = static_cast<const FreeListAllocator<U, sz>&>(other);
}

template <class T, std::size_t sz>
template <class U>
FreeListAllocator<T, sz>&
FreeListAllocator<T, sz>::operator=(FreeListAllocator<U, sz>&& other) noexcept
{
    return *this = static_cast<const FreeListAllocator<U, sz>&>(other);
}


template <class T, std::size_t sz>
FreeListAllocator<T, sz>::pointer FreeListAllocator<T, sz>::allocate(size_type n)
{
    if (n == 0)
        return nullptr;

    // throw std::bad_alloc?
    SIMU_ASSERT(n <= max_size(), "Use a bigger block size");

    for (auto& block : *blocks_)
    {
        pointer p = block.allocate<T>(n);
        if (p != nullptr)
            return p;
    }

    blocks_->emplace_back();
    return blocks_->back().allocate<T>(n);
}

template <class T, std::size_t sz>
void FreeListAllocator<T, sz>::deallocate(pointer p, size_type n) noexcept
{
    for (auto& block : *blocks_)
        if (block.deallocate(p, n))
            return;
}

template <class T, std::size_t sz>
constexpr FreeListAllocator<T, sz>::size_type
FreeListAllocator<T, sz>::max_size() const noexcept
{
    return (sz - alignof(T) + 1) / sizeof(T);
}


template <class T, std::size_t sz>
template <class U>
bool FreeListAllocator<T, sz>::operator==(const FreeListAllocator<U, sz>& other
) const noexcept
{
    return blocks_ == other.blocks_;
}

template <class T, std::size_t sz>
template <class U>
bool FreeListAllocator<T, sz>::operator!=(const FreeListAllocator<U, sz>& other
) const noexcept
{
    return !(*this == other);
}

} // namespace simu
