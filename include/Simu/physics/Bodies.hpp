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

#include <cmath>

#include <array>
#include <optional>
#include <ranges>

#include "Simu/config.hpp"

#include "Simu/physics/Body.hpp"

namespace std
{

template <class T, simu::Uint32 m, simu::Uint32 n>
bool isfinite(const simu::Matrix<T, m, n>& mat)
{
    bool isFinite = true;
    for (simu::Uint32 i = 0; i < mat.size(); ++i)
        isFinite = isFinite && std::isfinite(mat[i]);

    return isFinite;
}

} // namespace std

namespace simu
{


class SolverProxy
{
public:

    SolverProxy(Body* body)
        : centroid_{body->localProperties().centroid}, body_{body}
    {
        setPosition(body->position(), body->orientation());
        setVelocity(body->velocity(), body->angularVelocity());
    }

    void refresh() { *this = SolverProxy(this->body_); }

    void writeBack()
    {
        body_->position_    = position_;
        body_->orientation_ = orientation_;

        body_->velocity_        = velocity_;
        body_->angularVelocity_ = angularVelocity_;

        body_->toWorldSpace_ = toWorldSpace_;
        body_->collider_.update(body_->toWorldSpace_);

        body_->proxyIndex = Body::NO_INDEX;
    }

    void setPosition(Vec2 position, float orientation)
    {
        SIMU_ASSERT(std::isfinite(position), "");
        SIMU_ASSERT(std::isfinite(orientation), "");

        position_    = position;
        orientation_ = orientation;
        toWorldSpace_
            = Transform::transformAround(orientation_, position_, centroid_);
    }

    void setVelocity(Vec2 velocity, float angularVelocity)
    {
        SIMU_ASSERT(std::isfinite(velocity), "");
        SIMU_ASSERT(std::isfinite(angularVelocity), "");

        velocity_        = velocity;
        angularVelocity_ = angularVelocity;
    }

    Vec2  position() const { return position_; }
    float orientation() const { return orientation_; }

    Vec2  velocity() const { return velocity_; }
    float angularVelocity() const { return angularVelocity_; }

    Vec2      centroid() const { return toWorldSpace_ * centroid_; }
    Transform toWorldSpace() const { return toWorldSpace_; }

private:

    Vec2  position_;
    float orientation_;

    Vec2  velocity_;
    float angularVelocity_;

    Vec2      centroid_;
    Transform toWorldSpace_;

    Body* body_;
};

template <class T>
class ArrayView
{
public:

    ArrayView(T* begin, T* end) : begin_{begin}, end_{end} {}

    T*       begin() { return begin_; }
    const T* begin() const { return begin_; }

    T*       end() { return end_; }
    const T* end() const { return end_; }

    T&       operator[](std::size_t index) { return *(begin() + index); }
    const T& operator[](std::size_t index) const { return *(begin() + index); }

    std::size_t size() const { return end() - begin(); }

private:

    T* begin_;
    T* end_;
};


class Bodies
{
public:

    static constexpr Uint32 n = 2;

    typedef Vector<float, 3 * n> State;
    typedef State                Velocity;
    typedef State                Impulse;

    typedef Matrix<float, 3 * n, 3 * n> Mass;
    typedef Vector<float, 3 * n>        MassVec;
    typedef Vector<float, 3 * n>        Dominance;

    Bodies(
        std::initializer_list<Body*> bodies,
        std::optional<Dominance>     dominances = std::nullopt
    );


    static Bodies singleBody(Body* body)
    {
        Bodies b{
            {body, body},
            Dominance{1.f,  1.f }
        };

        b.isSingleBody = true;
        return b;
    }

    bool   isSolving() const { return isSolving_; }
    Uint32 size() const { return 2 - static_cast<Uint32>(isSingleBody); }

    // Constraint should never use the bodies() directly, except at construction.
    //  instead, call proxies() to retrieve the needed information.
    auto bodies()
    {
        assertNotSolving();
        Body** p = bodies_.data();
        return ArrayView(p, p + size());
    }
    auto bodies() const
    {
        assertNotSolving();
        Body const** p = const_cast<Body const**>(bodies_.data());
        return ArrayView(p, p + size());
    }

    auto proxies() const
    {
        assertHasProxies();
        SolverProxy const** p = const_cast<SolverProxy const**>(proxies_.data());
        return ArrayView(p, p + size());
    }


    bool isBodyStructural(const Body* body) const;

    void applyImpulse(const Impulse& impulse);
    void applyPositionCorrection(const State& correction);

    Velocity velocity() const;
    Mass     inverseMass() const { return Mass::diagonal(inverseMassVec()); }
    MassVec  inverseMassVec() const { return invMassVec_; }

    bool operator==(const Bodies& other) const
    {
        return bodies_ == other.bodies_;
    }

    bool operator!=(const Bodies& other) const
    {
        return bodies_ != other.bodies_;
    }

private:

    friend class Island;
    void startSolve(SolverProxy* start)
    {
        SIMU_ASSERT(bodies_[0]->proxyIndex != Body::NO_INDEX, "");
        SIMU_ASSERT(bodies_[1]->proxyIndex != Body::NO_INDEX, "");

        proxies_[0] = start + bodies_[0]->proxyIndex;
        proxies_[1] = start + bodies_[1]->proxyIndex;
        isSolving_  = true;
    }

    void endSolve()
    {
        proxies_[0] = nullptr;
        proxies_[1] = nullptr;
        isSolving_  = false;
    }

    void assertNotSolving() const
    {
        SIMU_ASSERT(
            !isSolving_,
            "Should not be accessed by constraints while they are being "
            "solved, use proxies()"
        );
    }

    void assertHasProxies() const
    {
        SIMU_ASSERT(
            isSolving_,
            "No proxies are currently associated with these bodies"
        );
    }

    std::array<Body*, n>        bodies_;
    std::array<SolverProxy*, n> proxies_{};

    MassVec invMassVec_;

    bool isSolving_   = false;
    bool isSingleBody = false;
};


} // namespace simu
