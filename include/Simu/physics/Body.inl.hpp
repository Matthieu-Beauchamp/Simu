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

#include "Simu/physics/Body.hpp"
#include "Simu/physics/World.hpp"

namespace simu
{

template <std::derived_from<Shape> S, class... Args>
Collider* Body::addCollider(const Material& material, Args&&... args)
{
    SIMU_ASSERT(world_ != nullptr, "Body must be owned by a World");

    Vec2 oldCentroid = centroid();

    Collider* c = colliders_.add<S>(this, material, std::forward<Args>(args)...);

    // see removeCollider.
    // Here we assume the new collider has the Body's velocity, ignoring
    //  conservation of momentums.
    Vec2 dvAtCentroid = angularVelocity() * perp(centroid() - oldCentroid);
    setVelocity(velocity() + dvAtCentroid);
    position_.setLocalCentroid(colliders_.properties().centroid);

    update();
    world_->addCollider(c);

    return c;
}

} // namespace simu
