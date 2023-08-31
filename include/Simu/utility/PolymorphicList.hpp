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

template <class T, mem::Allocator Alloc_>
class PolymorphicList
{
    template <bool>
    struct Iterator;

    struct Node;

    template <std::derived_from<T>>
    struct PolyNode;

public:

    typedef T                 value_type;
    typedef value_type*       pointer;
    typedef const value_type* const_pointer;
    typedef value_type&       reference;
    typedef const value_type& const_reference;


    typedef Iterator<false> iterator;
    typedef Iterator<true>  const_iterator;


    PolymorphicList(const Alloc_& alloc = Alloc_{}) : alloc_{alloc}
    {
        head_       = mem::allocate<Node>(alloc_, 1);
        head_->prev = head_; // will be the last element
        head_->next = head_; // will be the first element
    }

    PolymorphicList(const PolymorphicList&) = delete;

    PolymorphicList(PolymorphicList&&other){
        std::swap(head_, other.head_);

    }
    

    ~PolymorphicList()
    {
        clear();
        mem::destroy<Node>(alloc_, head_);
        mem::deallocate<Node>(alloc_, head_, 1);
    }

    iterator begin() { return iterator{head_->next}; }
    iterator end() { return iterator{head_}; }

    const_iterator begin() const { return const_iterator{head_->next}; }
    const_iterator end() const { return const_iterator{head_}; }

    template <std::derived_from<T> U, class... Args>
    iterator emplace_back(Args&&... args)
    {
        PolyNode<U>* node = allocate<U>();
        std::construct_at<U>(&node->data, std::forward<Args>(args)...);

        insert(node, head_);
        return iterator{node};
    }

    iterator erase(iterator it)
    {
        Node* node = it.node;
        Node* prev = node->prev;
        Node* next = node->next;

        prev->next = next;
        next->prev = prev;

        destroyAndDealloc(node);
        return iterator{next};
    }

    void clear()
    {
        auto it = begin();
        while (it != end())
            it = erase(it);
    }

private:

    typedef Uint8          Index;
    static constexpr Index maxIndex = std::numeric_limits<Index>::max();

    template <class U>
    using AllocatorFor = mem::ReboundTo<Alloc_, PolyNode<U>>;

    struct Node
    {
        Node* next = nullptr;
        Node* prev = nullptr;
        Index type;
    };

    template <std::derived_from<T> U>
    struct PolyNode : public Node
    {
        U data;
    };

    struct VTable
    {
        template <class U>
        static void destructorForU(Alloc_& alloc, Node* node)
        {
            auto derivedNode = static_cast<PolyNode<U>*>(node);

            mem::destroy<PolyNode<U>>(alloc, derivedNode);
            mem::deallocate<PolyNode<U>>(alloc, derivedNode, 1);
        }

        // is this needed?
        // I believe it is necessary in case we have:
        // class U : SomeClass, T
        // -> U* != T*
        template <class U>
        static const T* castForU(const Node* node)
        {
            auto derivedNode = static_cast<const PolyNode<U>*>(node);
            return static_cast<const T*>(&derivedNode->data);
        }

        template <class U>
        static VTable makeVTable()
        {
            VTable table;
            table.destructor = destructorForU<U>;
            table.cast       = castForU<U>;
            return table;
        }


        typedef void (*Destructor)(Alloc_& alloc, Node*);
        Destructor destructor;

        typedef const T* (*Cast)(const Node*);
        Cast cast;
    };


    template <bool isConst>
    class Iterator
    {
        typedef std::conditional_t<isConst, const Node*, Node*> NodePtr;

        friend Iterator<!isConst>;
        friend PolymorphicList;

    public:

        // TODO: Ensure we satisfy standard iterator requirements
        typedef std::ptrdiff_t difference_type;
        typedef std::conditional_t<isConst, const PolymorphicList::value_type, PolymorphicList::value_type>
            value_type;

        typedef std::conditional_t<isConst, PolymorphicList::const_pointer, PolymorphicList::pointer>
            pointer;
        typedef std::conditional_t<isConst, PolymorphicList::const_reference, PolymorphicList::reference>
            reference;

        typedef std::bidirectional_iterator_tag iterator_category;

        // iterator is invalid if n is nullptr, but is required for the std::semi_regular concept
        Iterator(NodePtr n = nullptr) : node{n} {}

        template <bool otherIsConst>
            requires(isConst || !otherIsConst)
        Iterator(const Iterator<otherIsConst>& other) : node{other.node}
        {
        }

        Iterator& operator++()
        {
            node = node->next;
            return *this;
        }

        Iterator operator++(int)
        {
            Iterator copy{*this};
            ++(*this);
            return copy;
        }

        Iterator& operator--()
        {
            node = node->prev;
            return *this;
        }

        Iterator operator--(int)
        {
            Iterator copy{*this};
            --(*this);
            return copy;
        }


        template <bool otherIsConst>
        bool operator==(const Iterator<otherIsConst>& other) const
        {
            return node == other.node;
        }

        template <bool otherIsConst>
        bool operator!=(const Iterator<otherIsConst>& other) const
        {
            return !(*this == other);
        }

        reference operator*() const { return *this->operator->(); }
        pointer   operator->() const
        {
            return const_cast<pointer>(tables_[node->type].cast(node));
        }

    private:

        NodePtr node;
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
            tables_[tableIndex] = VTable::template makeVTable<U>();
        }

        PolyNode<U>* polyNode = mem::allocate<PolyNode<U>>(alloc_, 1);
        polyNode->type        = tableIndex;
        return polyNode;
    }

    void destroyAndDealloc(Node* node)
    {
        tables_[node->type].destructor(alloc_, node);
    }

    void insert(Node* that, Node* beforeThis)
    {
        Node* prev = beforeThis->prev;
        prev->next = that;
        that->prev = prev;

        beforeThis->prev = that;
        that->next       = beforeThis;
    }

    static std::array<VTable, maxIndex> tables_;
    static Index                        nTables_;


    Alloc_ alloc_;
    Node*  head_;
};

template <class T,  mem::Allocator A>
std::array<typename PolymorphicList<T, A>::VTable, PolymorphicList<T, A>::maxIndex>
    PolymorphicList<T, A>::tables_{};

template <class T,  mem::Allocator A>
PolymorphicList<T, A>::Index PolymorphicList<T, A>::nTables_ = 0;

} // namespace simu
