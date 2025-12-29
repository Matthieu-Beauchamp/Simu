////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2025 Matthieu Beauchamp-Boulay
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
#include "Simu/physics-2.0/collision/broadphase/BoundingVolumeHierarchy.hpp"

namespace simu
{

namespace
{

template <std::size_t dimension>
std::size_t partition_along_dimension(
    std::vector<Entity>&      entities,
    std::vector<BoundingBox>& bounds,
    std::size_t               begin,
    std::size_t               end,
    float                     pivot
) noexcept {
    std::size_t i = begin;
    std::size_t j = end - 1;

    while (i < j) {
        while (i < j && bounds[i].center()[dimension] < pivot)
            ++i;

        while (i < j && bounds[j].center()[dimension] >= pivot)
            --j;

        std::swap(entities[i], entities[j]);
        std::swap(bounds[i], bounds[j]);
    }

    // Ensure at least one element on the left.
    // For example, when all centroids are equal (builds a linked list)
    if (i == begin) {
        return begin + 1;
    }

    return i; // pivot index
}

} // namespace


BoundingVolumeHierarchy BoundingVolumeHierarchy::mean_centroid_split(
    std::vector<Entity>      entities,
    std::vector<BoundingBox> bounds
) SIMU_NO_EXCEPT {
    struct Partition
    {
        std::size_t begin;
        std::size_t end;
        std::size_t index;
    };

    // Special cases to pass asserts if we have less than 2 objects
    if (entities.empty()) {
        return BoundingVolumeHierarchy();
    } else if (entities.size() == 1) {
        std::vector<Node> nodes{
            {bounds[0], internal::BvhNodeData::makeInternal(1, 0)},
            {bounds[0], internal::BvhNodeData::makeLeaf(entities[0].id())}
        };

        return BoundingVolumeHierarchy(std::move(nodes));
    }

    std::vector<Node> nodes;
    nodes.reserve(1 + entities.size() * 2);
    // Root always exists at index 0, child pointers are invalid if they point at index 0
    nodes.emplace_back(BoundingBox(), internal::BvhNodeData::makeInternal(0, 0));

    std::vector<Partition> partitions;
    partitions.emplace_back(0, entities.size(), 0);

    while (!partitions.empty()) {
        auto p = partitions.back();
        partitions.pop_back();

        std::size_t begin = p.begin;
        std::size_t end   = p.end;
        std::size_t index = p.index;

        SIMU_ASSERT(end - begin >= 2, "Should always have at least two elements");

        Vec2        mean         = Vec2();
        BoundingBox total_bounds = BoundingBox();

        for (std::size_t i = begin; i < end; ++i) {
            mean += bounds[i].center();
            total_bounds = total_bounds.combined(bounds[i]);
        }

        mean /= end - begin;

        auto        span_x = total_bounds.max()[0] - total_bounds.min()[0];
        auto        span_y = total_bounds.max()[1] - total_bounds.min()[1];
        std::size_t pivot;
        if (span_x > span_y) {
            pivot = partition_along_dimension<0>(entities, bounds, begin, end, mean[0]);
        } else {
            pivot = partition_along_dimension<1>(entities, bounds, begin, end, mean[1]);
        }

        SIMU_ASSERT(pivot > begin, "Partition should always have elements in the left");
        SIMU_ASSERT(pivot < end, "Partition should always have elements in the right");

        std::size_t left  = 0;
        std::size_t right = 0;

        if (end - pivot == 1) {
            right = nodes.size();
            nodes.emplace_back(
                bounds[pivot], internal::BvhNodeData::makeLeaf(entities[pivot].id())
            );
        } else {
            right = nodes.size();
            nodes.emplace_back(BoundingBox(), internal::BvhNodeData::makeInternal(0, 0));
            partitions.emplace_back(pivot, end, right);
        }

        if (pivot - begin == 1) {
            left = nodes.size();
            nodes.emplace_back(
                bounds[begin], internal::BvhNodeData::makeLeaf(entities[begin].id())
            );
        } else {
            left = nodes.size();
            nodes.emplace_back(BoundingBox(), internal::BvhNodeData::makeInternal(0, 0));
            partitions.emplace_back(begin, pivot, left);
        }

        nodes[index].bounds = total_bounds;
        nodes[index].data   = internal::BvhNodeData::makeInternal(
            static_cast<std::uint32_t>(left), static_cast<std::uint32_t>(right)
        );
    }

    return BoundingVolumeHierarchy(std::move(nodes));
}

BoundingVolumeHierarchy BoundingVolumeHierarchy::minimal_area_insertion(
    [[maybe_unused]] std::vector<Entity>      entities,
    [[maybe_unused]] std::vector<BoundingBox> bounds
) {
    NOT_IMPLEMENTED;
}

BoundingVolumeHierarchy BoundingVolumeHierarchy::morton_sort(
    [[maybe_unused]] std::vector<Entity>      entities,
    [[maybe_unused]] std::vector<BoundingBox> bounds
) {
    NOT_IMPLEMENTED;
}

BoundingVolumeHierarchy BoundingVolumeHierarchy::bottom_up_clustering(
    [[maybe_unused]] std::vector<Entity>      entities,
    [[maybe_unused]] std::vector<BoundingBox> bounds
) {
    NOT_IMPLEMENTED;
}

BoundingVolumeHierarchy::BoundingVolumeHierarchy()
    : nodes{
          {BoundingBox(), internal::BvhNodeData::makeInternal(0, 0)}
} {}

void BoundingVolumeHierarchy::insert([[maybe_unused]] Entity e, [[maybe_unused]] BoundingBox bounds) SIMU_NO_EXCEPT { NOT_IMPLEMENTED;
}

void BoundingVolumeHierarchy::remove([[maybe_unused]] Entity e) SIMU_NO_EXCEPT {
    NOT_IMPLEMENTED;
}

void BoundingVolumeHierarchy::collide(BoundingBox bounds, std::function<void(Entity)> callback) const noexcept {
    std::vector<std::size_t> stack{0};
    while (!stack.empty()) {
        auto node_index = stack.back();
        stack.pop_back();

        const Node& node = nodes[node_index];
        if (bounds.overlaps(node.bounds)) {
            if (node.data.isLeaf()) {
                callback(node.data.leaf());
            } else {
                if (node.data.left() != 0) {
                    stack.push_back(node.data.left());
                }
                if (node.data.right() != 0) {
                    stack.push_back(node.data.right());
                }
            }
        }
    }
}

void BoundingVolumeHierarchy::collide(
    const BoundingVolumeHierarchy&      other,
    std::function<void(Entity, Entity)> callback
) const noexcept {
    struct TreeComparison
    {
        std::size_t this_index;
        std::size_t other_index;
    };

    std::vector<TreeComparison> stack{
        {0, 0}
    };

    while (!stack.empty()) {
        auto comp = stack.back();
        stack.pop_back();

        const Node& this_node  = nodes[comp.this_index];
        const Node& other_node = other.nodes[comp.other_index];

        if (!this_node.bounds.overlaps(other_node.bounds)) {
            continue;
        }

        if (this_node.data.isLeaf() && other_node.data.isLeaf()) {
            callback(this_node.data.leaf(), other_node.data.leaf());
        } else if (this_node.data.isLeaf()) {
            if (other_node.data.left() != 0) {
                stack.emplace_back(comp.this_index, other_node.data.left());
            }
            if (other_node.data.right() != 0) {
                stack.emplace_back(comp.this_index, other_node.data.right());
            }
        } else if (other_node.data.isLeaf()) {
            if (this_node.data.left() != 0) {
                stack.emplace_back(this_node.data.left(), comp.other_index);
            }
            if (this_node.data.right() != 0) {
                stack.emplace_back(this_node.data.right(), comp.other_index);
            }
        } else {
            if (this_node.data.left() != 0 && other_node.data.left() != 0) {
                stack.emplace_back(this_node.data.left(), other_node.data.left());
            }
            if (this_node.data.left() != 0 && other_node.data.right() != 0) {
                stack.emplace_back(this_node.data.left(), other_node.data.right());
            }
            if (this_node.data.right() != 0 && other_node.data.left() != 0) {
                stack.emplace_back(this_node.data.right(), other_node.data.left());
            }
            if (this_node.data.right() != 0 && other_node.data.right() != 0) {
                stack.emplace_back(this_node.data.right(), other_node.data.right());
            }
        }
    }
}

} // namespace simu
