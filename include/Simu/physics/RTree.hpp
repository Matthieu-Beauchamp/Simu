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

#include <queue>
#include <utility>

#include "Simu/config.hpp"
#include "Simu/physics/BoundingBox.hpp"

#include "Simu/utility/View.hpp"
#include "Simu/utility/Callable.hpp"

namespace simu
{

// TODO: Cache friendliness, use a block allocator, or:
//  store nodes in a vector,
//  iterators have reference to tree and an index,
//  nodes use indices instead of pointers.

// TODO: If root has the same left and right child, we can remove some nullptr checks.

template <class T, class Allocator = std::allocator<T>>
class RTree
{
    typedef std::allocator_traits<Allocator> AllocTraits;

    template <bool>
    class Iterator;

public:

    typedef T        value_type;
    typedef T*       pointer;
    typedef const T* const_pointer;
    typedef T&       reference;
    typedef const T& const_reference;

    typedef Iterator<false> iterator;
    typedef Iterator<true>  const_iterator;

    RTree(const Allocator& alloc = Allocator{}) : alloc_{alloc}
    {
        root_ = makeNode(BoundingBox{});
    }

    ~RTree() { deleteNode(root_); }

    RTree(const RTree& other)            = delete;
    RTree& operator=(const RTree& other) = delete;

    RTree(RTree&& other) noexcept { *this = std::move(other); }
    RTree& operator=(RTree&& other) noexcept
    {
        std::swap(alloc_, other.alloc_);
        std::swap(nodeAlloc_, other.nodeAlloc_);
        std::swap(root_, other.root_);
        return *this;
    }

    template <Callable<void(iterator)> F>
    void forEachIn(BoundingBox box, const F& func)
    {
        forEachIn(box, func, root_);
    }

    template <Callable<void(iterator)> F>
    void forEachAt(Vec2 point, const F& func)
    {
        forEachIn(BoundingBox{point, point}, func);
    }

    template <Callable<void(iterator)> F>
    void forEachOverlapping(iterator it, const F& func)
    {
        forEachOverlapping(it.node, func);
    }

    iterator begin() { return iterator::leftmost(root_); }
    iterator end() { return iterator{root_}; }

    const_iterator begin() const { return const_iterator::leftmost(root_); }
    const_iterator end() const { return const_iterator{root_}; }


    iterator insert(BoundingBox bounds, const_reference val)
    {
        return emplace(bounds, val);
    }

    template <class... Args>
    iterator emplace(BoundingBox bounds, Args&&... args)
    {
        return insert(makeLeaf(bounds, std::forward<Args>(args)...));
    }

    iterator erase(iterator it)
    {
        iterator tmp = it++;
        erase(tmp.node);
        return it;
    }

    iterator update(iterator it, BoundingBox newBounds)
    {
        update(it.node, newBounds);
        return it;
    }

    void clear() { *this = RTree{}; }
    bool isEmpty() const
    {
        return root_->left == nullptr && root_->right == nullptr;
    }

    std::size_t size() const { return size(root_); }

    BoundingBox bounds() const { return root_->bounds; }
    BoundingBox bounds(const_iterator it) const { return it.bounds(); }

    Uint32 depth() const { return depth(root_) - 1; }

private:

    template <bool isConst>
    class Iterator;

    struct Node;

    typedef typename AllocTraits::template rebind_alloc<Node>  NodeAllocator;
    typedef typename AllocTraits::template rebind_traits<Node> NodeAllocTraits;

    Allocator     alloc_;
    NodeAllocator nodeAlloc_{alloc_};

    Node* root_;

    Node* makeNode(const BoundingBox& bounds = BoundingBox{})
    {
        Node* node = NodeAllocTraits::allocate(nodeAlloc_, 1);
        NodeAllocTraits::construct(nodeAlloc_, node, bounds);

        return node;
    }

    template <class... Args>
    Node* makeLeaf(const BoundingBox& bounds, Args&&... args)
    {
        pointer data = AllocTraits::allocate(alloc_, 1);
        AllocTraits::construct(alloc_, data, std::forward<Args>(args)...);

        Node* node = makeNode(bounds);
        node->data = data;

        return node;
    }

    void deleteNode(Node* node)
    {
        if (node == nullptr)
            return;

        deleteNode(node->left);
        deleteNode(node->right);

        if (node->data != nullptr)
        {
            AllocTraits::destroy(alloc_, node->data);
            AllocTraits::deallocate(alloc_, node->data, 1);
        }

        NodeAllocTraits::destroy(nodeAlloc_, node);
        NodeAllocTraits::deallocate(nodeAlloc_, node, 1);
    }

    void swapNodes(Node* first, Node* second)
    {
        std::swap(first->handle(), second->handle());
        std::swap(first->parent, second->parent);
    }

    void checkRotations(Node* grandParent)
    {
        // in case of grandParent being root when tree has 1 element
        if (grandParent->left == nullptr || grandParent->right == nullptr)
            return;

        if (!grandParent->right->isLeaf())
        {
            checkRotation(grandParent->left, grandParent->right->left);
            checkRotation(grandParent->left, grandParent->right->right);
        }
        if (!grandParent->left->isLeaf())
        {
            checkRotation(grandParent->right, grandParent->left->left);
            checkRotation(grandParent->right, grandParent->left->right);
        }
    }

    void checkRotation(Node* upper, Node* lower)
    {
        BoundingBox newBounds = lower->sibling()->bounds.combined(upper->bounds);
        if (area(newBounds) < area(lower->parent->bounds))
        {
            lower->parent->bounds = newBounds;
            swapNodes(lower, upper);
        }
    }

    void refit(Node* node)
    {
        // TODO: Benchmark and profile, we may not need to check rotations
        //  for all nodes whose bounds have changed.
        // Only check rotations on parent/grand parent of added leaf
        while (node != nullptr)
        {
            BoundingBox newBounds
                = bounds(node->left).combined(bounds(node->right));

            if (newBounds == node->bounds)
                return;

            node->bounds = newBounds;
            checkRotations(node);

            node = node->parent;
        }
    }

    // leaf should not currently be attached to anything
    void addLeafTo(Node* node, Node* leaf)
    {
        if (node->isLeaf())
        {
            Node* parent = makeNode(node->bounds.combined(leaf->bounds));

            node->replaceBy(parent);

            parent->setLeft(node);
            parent->setRight(leaf);
            refit(parent->parent);
        }
        else
        {
            if (node->left == nullptr)
                node->setLeft(leaf);
            else if (node->right == nullptr)
                node->setRight(leaf);
            else
            {
                Node* merged = makeNode(node->bounds);
                merged->setLeft(node->left);
                merged->setRight(node->right);
                node->setLeft(merged);
                node->setRight(leaf);
            }

            refit(node);
        }
    }


    float area(BoundingBox bounds)
    {
        if (!bounds.isValid())
            return 0.f;

        Vec2 stretch = bounds.max() - bounds.min();
        return stretch[0] * stretch[1];
    }

    BoundingBox bounds(Node* node)
    {
        if (node == nullptr)
            return BoundingBox{};

        return node->bounds;
    }

    iterator insert(Node* node)
    {
        // Based on:
        //  Fast Insertion-Based Optimization of Bounding Volume Hierarchies
        // from:
        //  COMPUTER GRAPHICS forum Volume 32 (2013), number 1 pp. 85–100
        // https://doi.org/10.1111/cgf.12000

        typedef std::pair<Node*, float> NodeCost;
        struct NodeCostCmp
        {
            bool operator()(const NodeCost& lhs, const NodeCost& rhs)
            {
                return lhs.second > rhs.second;
            }
        };

        // cheapest node at top
        std::priority_queue<NodeCost, std::vector<NodeCost>, NodeCostCmp> queue{};
        queue.emplace(root_, 0.f);

        NodeCost best = {nullptr, INFINITY};
        while (!queue.empty())
        {
            NodeCost current = queue.top();
            queue.pop();

            if (current.second + area(node->bounds) >= best.second)
                break;

            float directCost
                = area(current.first->bounds.combined(node->bounds));
            float totalCost = current.second + directCost;

            if (totalCost < best.second)
                best = {current.first, totalCost};

            float inducedCost = totalCost - area(current.first->bounds);
            if (inducedCost + area(node->bounds) < best.second)
            {
                if (current.first->left != nullptr)
                    queue.emplace(current.first->left, inducedCost);

                if (current.first->right != nullptr)
                    queue.emplace(current.first->right, inducedCost);
            }
        }

        addLeafTo(best.first, node);
        return iterator{node};
    }

    Node* extract(Node* node)
    {
        Node* sibling = node->sibling();

        if (node->parent->isRoot())
        {
            if (sibling == nullptr || sibling->isLeaf())
            {
                node->handle() = nullptr;
                refit(node->parent);
            }
            else
            {
                node->replaceBy(sibling->left);
                sibling->replaceBy(sibling->right);

                refit(node->parent);

                sibling->left  = nullptr;
                sibling->right = nullptr;
                deleteNode(sibling);
            }
        }
        else
        {
            sibling->handle() = nullptr;
            node->parent->replaceBy(sibling);
            refit(node->parent->parent);

            node->handle() = nullptr;
            deleteNode(node->parent);
        }

        return node;
    }

    void erase(Node* node) { deleteNode(extract(node)); }

    Uint32 depth(const Node* subRoot) const
    {
        if (subRoot == nullptr)
            return 0;

        return 1 + std::max(depth(subRoot->left), depth(subRoot->right));
    }

    std::size_t size(Node* node) const
    {
        if (node == nullptr)
            return 0;
        else if (node->isLeaf())
            return 1;
        else
            return size(node->left) + size(node->right);
    }

    template <class F>
    void forEachIn(const BoundingBox& box, const F& func, Node* node)
    {
        if (node == nullptr)
            return;

        if (box.overlaps(node->bounds))
        {
            if (node->isLeaf())
                func(iterator{node});
            else
            {
                forEachIn(box, func, node->left);
                forEachIn(box, func, node->right);
            }
        }
    }

    template <class F>
    void forEachOverlapping(Node* node, const F& func)
    {
        BoundingBox bounds = node->bounds;

        while (node->parent != nullptr)
        {
            Node* sibling = node->sibling();
            if (sibling != nullptr)
                forEachIn(bounds, func, sibling);

            node = node->parent;
        }
    }

    void update(Node* node, BoundingBox newBounds)
    {
        extract(node);
        node->bounds = newBounds;
        insert(node);
    }
};


} // namespace simu

#include "Simu/physics/RTree.inl.hpp"
