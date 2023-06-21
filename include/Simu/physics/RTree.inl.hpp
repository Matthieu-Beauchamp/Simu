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

#include "Simu/physics/RTree.hpp"

namespace simu
{

template <class T, class Allocator>
struct RTree<T, Allocator>::Node
{
    Node(const BoundingBox& bounds, pointer value = nullptr)
        : bounds{bounds}, data{value}
    {
    }

    bool isRoot() const { return parent == nullptr; }
    bool isLeaf() const { return data != nullptr; }

    bool isLeft() const { return !isRoot() && parent->left == this; }
    bool isRight() const { return !isRoot() && parent->right == this; }

    void setLeft(Node* node)
    {
        left         = node;
        node->parent = this;
    }
    void setRight(Node* node)
    {
        right        = node;
        node->parent = this;
    }
    void replaceChild(Node* child, Node* newChild)
    {
        child->handle()  = newChild;
        newChild->parent = this;
    }
    void replaceBy(Node* replacement)
    {
        parent->replaceChild(this, replacement);
    }

    Node*& childHandle(Node* child) { return child == left ? left : right; }

    Node*& otherChildHandle(Node* child)
    {
        return child != left ? left : right;
    }

    Node*& handle() { return parent->childHandle(this); }
    Node*& sibling() { return parent->otherChildHandle(this); }

    Node* parent = nullptr;
    Node* left   = nullptr;
    Node* right  = nullptr;

    BoundingBox bounds;
    pointer     data = nullptr;
};


template <class T, class Allocator>
template <bool isConst>
class RTree<T, Allocator>::Iterator
{
    typedef std::conditional_t<isConst, const Node*, Node*> NodePtr;

    typedef std::conditional_t<isConst, const_pointer, pointer> it_pointer;
    typedef std::conditional_t<isConst, const_reference, reference> it_reference;

    friend Iterator<!isConst>;
    friend RTree;

public:

    // TODO: Ensure we satisfy standard iterator requirements
    typedef std::conditional_t<isConst, const value_type, value_type> value_type;
    typedef std::ptrdiff_t difference_type;


    // behavior is undefined if n is nullptr, but is required for the std::semi_regular concept
    Iterator(NodePtr n = nullptr) : node{n} {}

    template <bool otherIsConst>
        requires(isConst || !otherIsConst)
    Iterator(const Iterator<otherIsConst>& other) : node{other.node}
    {
    }

    BoundingBox bounds() const { return node->bounds; }

    Iterator& operator++()
    {
        if (node->isLeft())
        {
            if (node->parent->right == nullptr)
                node = node->parent;
            else
            {
                node = node->parent->right;
                goLeft();
            }
        }
        else if (node->isRight())
        {
            node = node->parent;
            return ++*this;
        }

        return *this;
    }

    Iterator operator++(int)
    {
        Iterator copy{*this};
        ++*this;
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

    it_reference operator*() const { return *node->data; }
    it_pointer   operator->() const { return node->data; }

private:

    static Iterator leftmost(NodePtr n)
    {
        Iterator it{n};
        it.goLeft();
        return it;
    }

    void goLeft()
    {
        while (!node->isLeaf())
        {
            if (node->left != nullptr)
                node = node->left;
            else if (node->right != nullptr)
                node = node->right;
            else
                return;
        }
    }

    NodePtr node;
};

} // namespace simu
