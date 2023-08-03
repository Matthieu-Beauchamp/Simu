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

    SolverProxy() = default;
    SolverProxy(Position* p, Velocity* v) : position_{p}, velocity_{v} {}

    // void refresh() { *this = SolverProxy(this->body_); }

    // void writeBack()
    // {
    //     body_->position_ = *position_;
    //     body_->velocity_ = *velocity_;

    //     body_->update();

    //     body_->proxyIndex = Body::NO_INDEX;
    // }

    void advancePos(Vec2 dPos, float dTheta)
    {
        SIMU_ASSERT(std::isfinite(dPos), "");
        SIMU_ASSERT(std::isfinite(dTheta), "");

        position_->advance(dPos, dTheta);
    }

    void incrementVel(Vec2 dv, float dw)
    {
        setVelocity(velocity() + dv, angularVelocity() + dw);
    }

    void setVelocity(Vec2 velocity, float angularVelocity)
    {
        SIMU_ASSERT(std::isfinite(velocity), "");
        SIMU_ASSERT(std::isfinite(angularVelocity), "");

        velocity_->setLinear(velocity);
        velocity_->setAngular(angularVelocity);
    }

    Vec2  position() const { return position_->position(); }
    float orientation() const { return position_->orientation(); }

    Vec2  velocity() const { return velocity_->linear(); }
    float angularVelocity() const { return velocity_->angular(); }

    Vec2      centroid() const { return position_->centroid(); }
    Transform toWorldSpace() const { return position_->toWorldSpace(); }

private:

    friend class Bodies;
    friend class Proxies;

    Position* position_ = nullptr;
    Velocity* velocity_ = nullptr;
};


struct InvMasses
{
    float m0;
    float I0;
    float m1;
    float I1;
};

class Proxies
{
public:

    static constexpr Uint32 n = 2;

    typedef Vector<float, 3 * n> State;
    typedef State                VelocityVec;
    typedef State                Impulse;

    typedef Matrix<float, 3 * n, 3 * n> Mass;
    typedef Vector<float, 3 * n>        MassVec;
    typedef Vector<float, n>            Dominance;

    Proxies() = default;
    inline Proxies(
        const SolverProxy& p0,
        const SolverProxy& p1,
        const InvMasses&   masses
    );

    // TODO: Always iterates over 2 proxies, even if they point to the same body
    SolverProxy const* begin() const { return proxies_.data(); }
    SolverProxy const* end() const { return proxies_.data() + n; }

    SolverProxy* begin() { return proxies_.data(); }
    SolverProxy* end() { return proxies_.data() + n; }

    SolverProxy&       operator[](Uint32 index) { return proxies_[index]; }
    const SolverProxy& operator[](Uint32 index) const
    {
        return proxies_[index];
    }


    inline void applyImpulse(const Impulse& impulse);
    inline void applyPositionCorrection(const State& correction);

    inline VelocityVec velocity() const;


    const InvMasses& invMasses() const { return invMasses_; }
    MassVec          invMassVec() const
    {
        const InvMasses& m = invMasses();
        return MassVec{m.m0, m.m0, m.I0, m.m1, m.m1, m.I1};
    }
    Mass inverseMass() const { return Mass::diagonal(invMassVec()); }

private:


    std::array<SolverProxy, 2> proxies_;

    // TODO: This is duplicated
    InvMasses invMasses_;
};


class Bodies
{
public:

    static constexpr Uint32 n = 2;

    typedef Vector<float, n> Dominance;

    inline Bodies(
        std::initializer_list<Body*> bodies,
        std::optional<Dominance>     dominances = std::nullopt
    );

    static Bodies singleBody(Body* body)
    {
        Bodies b{
            {body, body},
            Dominance{1.f,  1.f }
        };

        return b;
    }


    Body const* const* begin() const
    {
        return const_cast<Body const* const*>(bodies_.data());
    }
    Body const* const* end() const
    {
        return const_cast<Body const* const*>(bodies_.data() + size());
    }

    Body* const* begin() { return bodies_.data(); }
    Body* const* end() { return bodies_.data() + size(); }

    Body*       operator[](Uint32 index) { return bodies_[index]; }
    const Body* operator[](Uint32 index) const { return bodies_[index]; }

    Uint32 size() const
    {
        return 1 + static_cast<Uint32>(bodies_[0] != bodies_[1]);
    }


    inline bool isBodyStructural(const Body* body) const;


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
    friend class Constraint;

    void startSolve(Position* ps, Velocity* vs)
    {
        Int32 index0 = bodies_[0]->proxyIndex;
        Int32 index1 = bodies_[1]->proxyIndex;

        SIMU_ASSERT(index0 != Body::NO_INDEX, "");
        SIMU_ASSERT(index1 != Body::NO_INDEX, "");

        proxies_ = Proxies{
            SolverProxy{ps + index0, vs + index0},
            SolverProxy{ps + index1, vs + index1},
            invMasses_
        };
    }

    void endSolve() { proxies_ = Proxies{}; }

    bool hasProxies() const { return proxies_[0].position_ != nullptr; }

    Proxies& getProxies()
    {
        SIMU_ASSERT(hasProxies(), "has no proxies");
        return proxies_;
    }

    std::array<Body*, n> bodies_;

    Proxies proxies_{};

    InvMasses invMasses_;
};


} // namespace simu

#include "Simu/physics/Bodies.inl.hpp"
