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

#include <cstdlib>
#include <cstddef>
#include <memory>
#include <functional>
#include <list>

#include "Simu/config.hpp"

namespace simu
{

template <class Alloc, class T>
using ReboundTo = typename std::allocator_traits<Alloc>::template rebind_alloc<T>;

// otherwise Deleters of derived virtual types cannot be assigned
//  when moving unique pointers...
typedef std::function<void(void*)> Deleter;

template <class T>
using UniquePtr = std::unique_ptr<T, Deleter>;


// unique pointer may outlive this allocator.
template <class T, class Alloc, class... Args, std::enable_if_t<!std::is_array_v<T>, bool> = true>
UniquePtr<T> makeUnique(const Alloc& alloc, Args&&... args)
{
    typedef ReboundTo<Alloc, T> A;
    A                           a{alloc};

    Deleter d = [=](void* obj) mutable {
        T* typed = static_cast<T*>(obj);
        std::allocator_traits<A>::destroy(a, typed);
        std::allocator_traits<A>::deallocate(a, typed, 1);
    };

    T* obj = std::allocator_traits<A>::allocate(a, 1);
    std::allocator_traits<A>::construct(a, obj, std::forward<Args>(args)...);
    return {obj, d};
}

template <
    class T,
    class Alloc,
    class... Args,
    std::enable_if_t<std::is_array_v<T> && std::extent_v<T> == 0, bool> = true>
UniquePtr<T> makeUnique(const Alloc& alloc, std::size_t size)
{
    typedef std::remove_extent_t<T> U;
    typedef ReboundTo<Alloc, U>     A;
    A                               a{alloc};

    Deleter d = [=](void* arr) mutable {
        U* typed = static_cast<T*>(arr);

        for (std::size_t i = 0; i < size; ++i)
            std::allocator_traits<A>::destroy(a, typed + i);

        std::allocator_traits<A>::deallocate(a, typed, size);
    };

    T* arr = std::allocator_traits<A>::allocate(a, size);

    for (std::size_t i = 0; i < size; ++i)
        std::allocator_traits<A>::construct(a, arr + i);

    return {arr, d};
}

template <
    class T,
    class Alloc,
    class... Args,
    std::enable_if_t<std::is_array_v<T> && std::extent_v<T> != 0, bool> = true>
UniquePtr<T> makeUnique(const Alloc& alloc, std::size_t size) = delete;


template <class Container, class A>
void replaceAllocator(Container& c, const A& alloc)
{
    static_assert(
        std::allocator_traits<A>::propagate_on_container_move_assignment::value,
        "This will not replace the allocator"
    );

    if (alloc == c.get_allocator())
        return;

    c = Container{c.begin(), c.end(), alloc};
    SIMU_ASSERT(c.get_allocator() == alloc, "Allocator was not replaced");
}

// TODO: Could keep track of the biggest available slot to avoid searches when
//  size + (alignement-1) > slotSize
class Block
{
public:

    Block(std::size_t size) : data_{new Byte[size]}, sz{size}
    {
        if (data_ == nullptr)
            throw std::bad_alloc{};

        freeList_.emplace_back(data_, data_ + sz);
    }

    Block(const Block&) = delete;
    Block(Block&&)      = delete;

    ~Block() { delete[] data_; }

    template <class T>
    T* allocate(std::size_t n);

    template <class T>
    bool deallocate(T* p, std::size_t n) noexcept;

    bool isEmpty() const noexcept
    {
        return freeList_.size() == 1 && freeList_.back().size() == sz;
    }

private:

    typedef std::uint8_t Byte;

    struct FreeBlock
    {
        FreeBlock(Byte* start, Byte* end) : start{start}, end{end} {}
        std::size_t size() const noexcept { return end - start; }

        Byte* start;
        Byte* end;
    };

    std::list<FreeBlock> freeList_{};

    Byte*       data_ = nullptr;
    std::size_t sz;
};


// https://github.com/mtrebi/memory-allocators
// https://en.cppreference.com/w/cpp/named_req/Allocator
template <class T, std::size_t blockSize = 32 * 1024>
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

    FreeListAllocator(const FreeListAllocator& other) noexcept;
    FreeListAllocator& operator=(const FreeListAllocator& other) noexcept;

    template <class U>
    FreeListAllocator(const FreeListAllocator<U, blockSize>& other) noexcept;


    FreeListAllocator select_on_container_copy_construction() const noexcept
    {
        return *this;
    }

    typedef std::true_type propagate_on_container_copy_assignment;
    typedef std::true_type propagate_on_container_move_assignment;
    typedef std::true_type propagate_on_container_swap;

    pointer allocate(size_type n);
    void    deallocate(pointer p, size_type n) noexcept;

    template <class U>
    bool operator==(const FreeListAllocator<U, blockSize>& other) const noexcept;

    template <class U>
    bool operator!=(const FreeListAllocator<U, blockSize>& other) const noexcept;

private:

    template <class U, std::size_t s>
    friend class FreeListAllocator;

    std::shared_ptr<std::list<Block>> blocks_;
};

////////////////////////////////////////////////////////////
// Block
////////////////////////////////////////////////////////////

template <class T>
T* Block::allocate(std::size_t n)
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

template <class T>
bool Block::deallocate(T* p, std::size_t n) noexcept
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
    bool canMergeWithPrev = (next != freeList_.begin() && std::prev(next)->end == start);

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

template <class T, std::size_t sz>
FreeListAllocator<T, sz>::FreeListAllocator()
    : blocks_{std::make_shared<std::list<Block>>()}
{
}

template <class T, std::size_t sz>
FreeListAllocator<T, sz>::FreeListAllocator(const FreeListAllocator& other) noexcept
{
    *this = other;
}

template <class T, std::size_t sz>
FreeListAllocator<T, sz>&
FreeListAllocator<T, sz>::operator=(const FreeListAllocator& other) noexcept
{
    blocks_ = other.blocks_;
    return *this;
}

template <class T, std::size_t sz>
template <class U>
FreeListAllocator<T, sz>::FreeListAllocator(const FreeListAllocator<U, sz>& other
) noexcept
    : blocks_{other.blocks_}
{
}

template <class T, std::size_t sz>
typename FreeListAllocator<T, sz>::pointer
FreeListAllocator<T, sz>::allocate(size_type n)
{
    if (n == 0)
        return nullptr;

    for (auto& block : *blocks_)
    {
        pointer p = block.template allocate<T>(n);
        if (p != nullptr)
            return p;
    }

    std::size_t allocSize = n * sizeof(T) + alignof(T) - 1;

    blocks_->emplace_back(std::max(sz, allocSize));
    return blocks_->back().template allocate<T>(n);
}

template <class T, std::size_t sz>
void FreeListAllocator<T, sz>::deallocate(pointer p, size_type n) noexcept
{
    for (auto it = blocks_->begin(); it != blocks_->end(); ++it)
    {
        auto& block = *it;
        if (block.deallocate(p, n))
        {
            if (block.isEmpty())
                blocks_->erase(it);

            return;
        }
    }
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
