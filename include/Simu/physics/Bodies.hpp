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

#include <array>
#include <optional>
#include <ranges>

#include "Simu/config.hpp"
#include "Simu/math/Matrix.hpp"

#include "Simu/physics/ConstraintInterfaces.hpp"

namespace simu
{

// TODO: Ensure the array is immutable, 
//  const applies to the stored bodies.

template <Uint32 n>
class Bodies : public std::array<PhysicsBody*, n>
{
    typedef std::array<PhysicsBody*, n> Base;

public:

    typedef Vector<float, 3 * n> State;
    typedef State                Velocity;
    typedef State                Impulse;

    typedef Matrix<float, 3 * n, 3 * n> Mass;
    typedef Vector<float, n>            Dominance;

    Bodies(
        std::initializer_list<PhysicsBody*> bodies,
        std::optional<Dominance>            dominances = std::nullopt
    );

    BodiesView view()
    {
        return BodiesView{this->data(), this->data() + this->size()};
    }
    ConstBodiesView view() const
    {
        return ConstBodiesView{
            const_cast<PhysicsBody const**>(this->data()),
            const_cast<PhysicsBody const**>(this->data()) + this->size()};
    }

    bool isBodyStructural(const PhysicsBody* body) const;

    void applyImpulse(const Impulse& impulse);
    void applyPositionCorrection(const State& correction);

    Velocity velocity() const;
    Mass     inverseMass() const;

private:

    Dominance dominances_;
};


} // namespace simu

#include "Simu/physics/Bodies.inl.hpp"
