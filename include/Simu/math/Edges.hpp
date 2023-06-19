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

#include "Simu/math/Geometry.hpp"

namespace simu
{

template <Geometry T>
class Edges
{
    typedef IteratorOf<const T> It;

public:

    Edges(const T& geometry) : geometry_{geometry} {}

    class Edge
    {
    public:

        Edge(It from, It to) : from_{from}, to_{to} {}

        Vec2 from() const { return *from_; }
        Vec2 to() const { return *to_; }

        It fromIt() const { return from_; }
        It toIt() const { return to_; }

        Vec2 direction() const { return to() - from(); }

        // if the geometry is positively oriented, the normal points outwards
        Vec2 normal() const { return perp(direction(), true); }
        Vec2 normalizedNormal() const { return normalized(normal()); }

        float distanceToPoint(Vec2 point) const
        {
            return std::abs(dot(normalizedNormal(), to() - point));
        };

        float distanceToOrigin() const { return distanceToPoint(Vec2{}); };

        // requires that this is from positively oriented geometry,
        // Here this is considered an infinite line
        // TODO: The name produces confusing lines
        bool isOutside(Vec2 p) const { return dot(normal(), p - to()) > 0.f; }
        bool isInside(Vec2 p) const { return dot(-normal(), p - to()) > 0.f; }

        // requires that the geometry be positively oriented.
        // TODO: Incomplete...
        // if the edges are parallel, assumes they overlap and return this->from()
        // otherwise assumes this intersects the infinite line of other and returns the point of intersection
        Vertex clipInside(const Edge& other) const
        {
            if (dot(direction(), other.normal()) == 0.f)
                return from();

            if (other.isInside(from()) && other.isInside(to()))
            {
                if (other.distanceToPoint(from()) < other.distanceToPoint(to()))
                    return from();
                else
                    return to();
            }

            Vec2 parametricCoefficients = solve(
                Mat2::fromCols({direction(), other.direction()}),
                other.from() - from()
            );

            SIMU_ASSERT(
                Interval<float>(0.f, 1.f).contains(parametricCoefficients[0]),
                "Intersection is not within this edge, edges were assumed to "
                "be intersecting"
            );

            return from() + parametricCoefficients[0] * direction();
        }

        bool operator==(const Edge& other) const = default;
        bool operator!=(const Edge& other) const = default;

    private:

        friend class Iterator;
        friend Edges;

        It from_;
        It to_;
    };

    class Iterator
    {
    public:

        Iterator(Edge e) : e_{e} {}

        Iterator& operator++()
        {
            e_.from_ = e_.to_++;
            return *this;
        }

        Iterator operator++(int)
        {
            Iterator copy{*this};
            ++*this;
            return copy;
        }

        bool operator==(const Iterator& other) const { return e_ == other.e_; }
        bool operator!=(const Iterator& other) const { return e_ != other.e_; }

        const Edge& operator*() const { return e_; }
        const Edge* operator->() const { return &e_; }

    private:

        Edge e_;
    };

    Iterator begin() const
    {
        return Iterator{
            Edge{std::prev(geometry_.end()), geometry_.begin()}
        };
    }

    Iterator end() const
    {
        return Iterator{
            Edge{std::prev(geometry_.end()), geometry_.end()}
        };
    }


    Iterator next(Iterator edge) const
    {
        ++edge;
        return (edge == end()) ? begin() : edge;
    }

    Iterator previous(Iterator edge) const
    {
        if (edge->from_ == geometry_.begin())
            return begin();

        return Iterator{
            Edge{std::prev(edge->from_), edge->from_}
        };
    }

private:

    const T& geometry_;
};


template <Geometry T>
Edges<T> edgesOf(const T& geometry)
{
    return Edges<T>{geometry};
}


} // namespace simu
