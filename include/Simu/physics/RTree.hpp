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
#include "Simu/utility/Algo.hpp"
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

    struct Node;

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

    // TODO: Request a general range instead of a view of iterator*
    template <Callable<void(iterator, iterator)> F>
    void forEachOverlapping(const ViewType<iterator*>& iterators, const F& func)
    {
        OverlapList list{iterators};
        std::size_t middle = list.middle();

        if (root_->left != nullptr)
        {
            list.sortOverlapping(root_->left->bounds);
            forEachOverlapping(root_->left, list, func);
        }

        if (root_->right != nullptr)
        {
            list.setMiddle(middle);
            list.sortOverlapping(root_->right->bounds);
            forEachOverlapping(root_->right, list, func);
        }
    }

    template <
        Callable<BoundingBox(iterator)>    Update,
        Callable<void(iterator, iterator)> OnCollision>
    void updateAndCollide(const Update& update, const OnCollision& onCollision)
    {
        // Using the RTree's allocator will result in fragmented Node allocations
        //  and drastically decrease performance.
        Allocator updateAlloc{};

        std::vector<Node*, ReboundTo<Allocator, Node*>> nodes{updateAlloc};
        for (iterator it = this->begin(); it != this->end(); ++it)
        {
            nodes.emplace_back(it.node);
            it.node->bounds = update(it);
        }

        for (Node* node : nodes)
            node->handle() = nullptr;

        if (nodes.size() == 0)
        {
            return;
        }
        else if (nodes.size() == 1)
        {
            insert(nodes[0]);
        }
        else
        {
            RecycleBin bin{*this, updateAlloc};
            bin.recycleSubTree(root_);

            OverlapList collisions{updateAlloc};
            for (Node* node : nodes)
                collisions.addOverlapping(node);

            root_ = updateAndCollide(
                makeView(nodes.data(), nodes.data() + nodes.size()),
                std::move(collisions),
                onCollision,
                bin
            );
        }
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

    typedef typename AllocTraits::template rebind_alloc<Node>  NodeAllocator;
    typedef typename AllocTraits::template rebind_traits<Node> NodeAllocTraits;

    class RecycleBin
    {
    public:

        typedef ReboundTo<Allocator, Node*> Allocator;

        RecycleBin(RTree& tree, Allocator alloc) : bin_{alloc}, tree_{tree} {}
        ~RecycleBin()
        {
            for (Node* node : bin_)
                tree_.deleteNode(node);
        }

        void recycleSubTree(Node* subRoot)
        {
            if (subRoot->left != nullptr)
                recycleSubTree(subRoot->left);
            if (subRoot->right != nullptr)
                recycleSubTree(subRoot->right);

            recycle(subRoot);
        }

        void recycle(Node* node)
        {
            node->parent = nullptr;
            node->left   = nullptr;
            node->right  = nullptr;
            bin_.emplace_back(node);
        }

        Node* getNode()
        {
            if (!bin_.empty())
            {
                Node* node = bin_.back();
                bin_.pop_back();
                return node;
            }

            return tree_.makeNode();
        }

    private:

        std::vector<Node*, Allocator> bin_;
        RTree&                        tree_;
    };

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
            BoundingBox newBounds = bounds(node->left)
                                        .combined(bounds(node->right));

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

            float directCost = area(current.first->bounds.combined(node->bounds));
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

    class OverlapList
    {
    public:

        typedef ReboundTo<Allocator, Node*> Allocator;

        OverlapList(const Allocator& alloc = Allocator{}) : nodes_{alloc} {}

        OverlapList(
            const ViewType<iterator*>& iterators,
            const Allocator&           alloc = Allocator{}
        )
            : OverlapList{alloc}
        {
            nodes_.reserve(iterators.size());
            for (iterator it : iterators)
                nodes_.emplace_back(it.node);

            middle_ = nodes_.size();
        }

        OverlapList(const OverlapList& other)
            : nodes_{other.nodes_.get_allocator()}
        {
            for (Node* node : other.overlapping())
                nodes_.emplace_back(node);

            middle_ = nodes_.size();
        }

        OverlapList(OverlapList&& other) = default;

        void addOverlapping(Node* node)
        {
            nodes_.emplace_back(node);
            ++middle_;
        }

        void sortOverlapping(const BoundingBox& box)
        {
            auto mid = booleanSort(
                nodes_.data(),
                nodes_.data() + middle_,
                [=](Node** it) { return (*it)->bounds.overlaps(box); }
            );

            middle_ = mid - nodes_.data();
        }

        auto overlapping()
        {
            return makeView(nodes_.data(), nodes_.data() + middle_);
        }
        auto overlapping() const
        {
            return makeView(nodes_.data(), nodes_.data() + middle_);
        }

        void        setMiddle(std::size_t middle) { middle_ = middle; }
        std::size_t middle() const { return middle_; }

    private:

        std::vector<Node*, Allocator> nodes_{};
        std::size_t                   middle_{};
    };

    // Adapted from https://github.com/mtsamis/box2d-optimized
    template <class F>
    void forEachOverlapping(Node* subRoot, OverlapList& list, const F& func)
    {
        if (subRoot->isLeaf())
        {
            for (Node* it : list.overlapping())
                func(iterator{subRoot}, iterator{it});
        }
        else
        {
            std::size_t middle = list.middle();

            list.sortOverlapping(subRoot->left->bounds);
            forEachOverlapping(subRoot->left, list, func);

            list.setMiddle(middle);

            list.sortOverlapping(subRoot->right->bounds);
            forEachOverlapping(subRoot->right, list, func);
        }
    }

    // Adapted from https://github.com/mtsamis/box2d-optimized
    template <class OnCollision>
    Node* updateAndCollide(
        ViewType<Node**>   nodes,
        OverlapList        collisions,
        const OnCollision& onCollision,
        RecycleBin&        bin
    )
    {
        std::size_t count = nodes.size();

        if (count == 0)
            return nullptr;
        else if (count == 1)
        {
            for (Node* node : collisions.overlapping())
                onCollision(iterator{nodes[0]}, iterator{node});

            return nodes[0];
        }

        Node* subRoot = bin.getNode();

        BoundingBox centerBounds{};
        for (Node* node : nodes)
        {
            Vec2 c       = node->bounds.center();
            centerBounds = centerBounds.combined(BoundingBox{c, c});
        }

        Uint32 splitAxis;
        float  split;
        {
            Vec2 dim  = centerBounds.max() - centerBounds.min();
            splitAxis = dim[0] > dim[1] ? 0 : 1;
            split     = centerBounds.center()[splitAxis];
        }

        Node** middle = booleanSort(nodes.begin(), nodes.end(), [=](Node** it) {
            return (*it)->bounds.center()[splitAxis] < split;
        });


        // prevent degenerate linear trees.
        {
            std::ptrdiff_t minNodes = std::max(count / 16, (std::size_t)1);
            if (middle - nodes.begin() < minNodes)
                middle = nodes.begin() + minNodes;
            else if (nodes.end() - middle < minNodes)
                middle = nodes.end() - minNodes;
        }

        ViewType<Node**> left  = makeView(nodes.begin(), middle);
        ViewType<Node**> right = makeView(middle, nodes.end());

        auto constructBounds = [](ViewType<Node**> nodes) {
            BoundingBox b{};
            for (Node* n : nodes)
                b = b.combined(n->bounds);

            return b;
        };

        BoundingBox leftBounds  = constructBounds(left);
        BoundingBox rightBounds = constructBounds(right);
        subRoot->bounds         = leftBounds.combined(rightBounds);

        OverlapList leftCollisions{collisions};
        leftCollisions.sortOverlapping(leftBounds);

        OverlapList rightCollisions{std::move(collisions)};
        rightCollisions.sortOverlapping(rightBounds);

        if (leftBounds.overlaps(rightBounds))
        {
            if (left.size() < right.size())
            {
                for (Node* node : left)
                    if (rightBounds.overlaps(node->bounds))
                        rightCollisions.addOverlapping(node);
            }
            else
            {
                for (Node* node : right)
                    if (leftBounds.overlaps(node->bounds))
                        leftCollisions.addOverlapping(node);
            }
        }

        subRoot->left = updateAndCollide(
            left,
            std::move(leftCollisions),
            onCollision,
            bin
        );

        subRoot->right = updateAndCollide(
            right,
            std::move(rightCollisions),
            onCollision,
            bin
        );

        if (subRoot->left != nullptr)
            subRoot->left->parent = subRoot;
        if (subRoot->right != nullptr)
            subRoot->right->parent = subRoot;

        return subRoot;
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
