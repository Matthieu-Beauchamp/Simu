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
    std::size_t j = end;

    while (i < j) {
        while (i < j && bounds[begin].center()[dimension] < pivot)
            ++i;

        while (i < j && bounds[end].center()[dimension] >= pivot)
            --j;

        std::swap(entities[begin], entities[end]);
        std::swap(bounds[begin], bounds[end]);
    }

    return begin; // pivot index
}

} // namespace


BoundingVolumeHierarchy BoundingVolumeHierarchy::mean_centroid_split(
    std::vector<Entity>      entities,
    std::vector<BoundingBox> bounds
) {
    struct Partition
    {
        std::size_t begin;
        std::size_t end;
        std::size_t index;
    };


    std::vector<Node> nodes;
    nodes.reserve(entities.size() * 2);
    // Root always exists at index 0, child pointers are invalid if they point at index 0
    nodes.emplace_back(BoundingBox(), internal::BvhNodeData::makeInternal(1, 1));

    std::vector<Partition> partitions;
    partitions.emplace_back(0, entities.size(), 0);

    while (!partitions.empty()) {
        auto p = partitions.back();
        partitions.pop_back();

        std::size_t begin = p.begin;
        std::size_t end   = p.end;
        std::size_t index = p.index;

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

        // TODO: Is progress garanteed?

        // TODO: Check off by 1 errors...

        if (pivot == begin) {
            pivot++;
        }

        std::size_t left = 0;
        std::size_t right = 0;
        if (pivot - begin == 1) {
            left = nodes.size();
            nodes.emplace_back(bounds[begin], internal::BvhNodeData::makeLeaf(entities[begin].id()));
        } else {
            left = nodes.size();
            nodes.emplace_back(BoundingBox(), internal::BvhNodeData::makeInternal(0, 0));
            partitions.emplace_back(begin, pivot, left);
        }

        if (end - pivot == 1) {
            right = nodes.size();
            nodes.emplace_back(bounds[pivot], internal::BvhNodeData::makeLeaf(entities[pivot].id()));
        } else {
            right = nodes.size();
            nodes.emplace_back(BoundingBox(), internal::BvhNodeData::makeInternal(0, 0));
            partitions.emplace_back(pivot, end, right);
        }

        nodes[index].bounds = total_bounds;
        nodes[index].data   = internal::BvhNodeData::makeInternal(left, right);
    }

    return BoundingVolumeHierarchy(std::move(nodes));
}

BoundingVolumeHierarchy BoundingVolumeHierarchy::minimal_area_insertion(
    std::vector<Entity>      entities,
    std::vector<BoundingBox> bounds
) {
    NOT_IMPLEMENTED;
}

BoundingVolumeHierarchy
BoundingVolumeHierarchy::morton_sort(std::vector<Entity> entities, std::vector<BoundingBox> bounds) {
    NOT_IMPLEMENTED;
}

BoundingVolumeHierarchy BoundingVolumeHierarchy::bottom_up_clustering(
    std::vector<Entity>      entities,
    std::vector<BoundingBox> bounds
) {
    NOT_IMPLEMENTED;
}

void BoundingVolumeHierarchy::insert(Entity e, BoundingBox bounds) {}
void BoundingVolumeHierarchy::remove(Entity e) {}

void BoundingVolumeHierarchy::collide(BoundingBox bounds, std::function<void(Entity)> callback) const {
}

std::vector<Entity> BoundingVolumeHierarchy::collide(BoundingBox bounds) const {}

void BoundingVolumeHierarchy::collide(
    const BoundingVolumeHierarchy&      other,
    std::function<void(Entity, Entity)> callback
) const {}

} // namespace simu
