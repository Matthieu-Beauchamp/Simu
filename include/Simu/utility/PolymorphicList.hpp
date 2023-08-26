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

#include <vector>

#include "Simu/config.hpp"
#include "Simu/utility/Memory.hpp"

namespace simu
{

// template <class T, class Alloc>
// using PolymorphicList = std::vector<UniquePtr<T>, ReboundTo<Alloc, UniquePtr<T>>>;

template <class T, class Alloc_>
class PolymorphicList
{
public:

private:

    typedef Uint8          Index;
    static constexpr Index maxIndex = std::numeric_limits<Index>::max();

    template <class U>
    using AllocatorFor = ReboundTo<Alloc_, PolyNode<U>>;

    struct Node
    {
        Node* next = nullptr;
        Node* prev = nullptr;
        Index type;
    }

    template <std::derived_from<T> U>
    struct PolyNode : public Node
    {
        U data;
    };

    struct VTable
    {
        template <class U>
        static void destructorForU(const Alloc_& alloc, Node* node)
        {
            auto derivedNode = static_cast<PolyNode<U>*>(node);

            std::allocator_traits<AllocatorFor<U>>::destroy(alloc, derivedNode);
            std::allocator_traits<AllocatorFor<U>>::deallocate(alloc, derivedNode);
        }

        // is this needed?
        // I believe it is necessary in case we have:
        // class U : SomeClass, T
        // -> U* != T*
        template <class U>
        static T* castForU(Node* node)
        {
            auto derivedNode = static_cast<PolyNode<U>*>(node);
            return static_cast<T*>(&derivedNode->data);
        }

        template <class U>
        static VTable makeVTable()
        {
            VTable table;
            table.destructor = destructorForU<U>;
            table.cast       = castForU<U>;
        }


        typedef void (*Destructor)(const Alloc_& alloc, Node*);
        Destructor destructor;

        typedef T* (*Cast)(Node*);
        Cast cast;
    };

    template <class U>
    PolyNode<U>* allocate()
    {
        static Index tableIndex = maxIndex;

        bool isFirstCall = tableIndex == maxIndex;
        if (isFirstCall)
        {
            SIMU_ASSERT(
                nTables_ < maxIndex, "Too many classes in this polymorphic list"
            );

            tableIndex          = nTables_++;
            tables_[tableIndex] = VTable::makeVTable<U>();
        }

        PolyNode<U>* polyNode = std::allocator_traits<AllocatorFor<U>>::allocate(
            alloc_, 1
        );
        polyNode->type = tableIndex;
        return polyNode;
    }

    void erase(Node* node) { tables_[node->type].destructor(alloc_, node); }

    static std::array<VTable, maxIndex> tables_;
    static Index                        nTables_ = 0;


    Alloc_ alloc_;
};


} // namespace simu
