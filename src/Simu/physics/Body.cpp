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

#include "Simu/physics/Body.hpp"

#include "Simu/physics/Constraint.hpp"
#include "Simu/physics/World.hpp"

namespace simu
{

bool Body::interactsAsStructural() const
{
    bool isStruct = all(velocity() == Vec2{}) && angularVelocity() == 0.f;
    for (Constraint* constraint : constraints_)
        isStruct = isStruct && constraint->bodies().isBodyStructural(this);
    for (ContactConstraint* constraint : contacts_)
        isStruct = isStruct && constraint->bodies().isBodyStructural(this);

    return isStruct;
}

Collider* Body::addCollider(const ColliderDescriptor& descriptor)
{
    SIMU_ASSERT(world_ != nullptr, "Body must be owned by a World");

    Vec2 oldCentroid = centroid();

    Collider* c = colliders_.add(descriptor, this);

    // see removeCollider.
    // Here we assume the new collider has the Body's velocity, ignoring
    //  conservation of momentums.
    Vec2 dvAtCentroid = angularVelocity() * perp(centroid() - oldCentroid);
    setVelocity(velocity() + dvAtCentroid);
    position_.setLocalCentroid(localCentroid());

    update();
    world_->addCollider(c);

    return c;
}

void Body::removeCollider(Collider* collider)
{
    Vec2 oldCentroid = centroid();

    world_->removeCollider(collider);
    colliders_.remove(collider);
    if (colliders_.isEmpty())
        return;

    // Assume the removed collider still exists and keeps its portion
    //  of the Body's momentum.
    //  => velocities don't change for the Body, except at the center of mass

    Vec2 dvAtCentroid = angularVelocity() * perp(centroid() - oldCentroid);
    setVelocity(velocity() + dvAtCentroid);
    position_.setLocalCentroid(localCentroid());

    update();
}

} // namespace simu
