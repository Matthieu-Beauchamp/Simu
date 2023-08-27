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

#include "Simu/physics/World.hpp"
#include "Simu/physics/Collider.hpp"

namespace simu
{

template <std::derived_from<Body> T, class... Args>
T* World::makeBody(Args&&... args)
{
    T* b = static_cast<T*>(
        &*bodies_.emplace_back<T>(this, bAlloc_, std::forward<Args>(args)...)
    );
    b->onConstruction(*this);

    return b;
}

template <std::derived_from<Constraint> T, class... Args>
T* World::makeConstraint(Args&&... args)
{
    T* c = static_cast<T*>(&*constraints_.emplace_back<T>(std::forward<Args>(args
    )...));
    c->onConstruction(*this);

    for (Body* body : c->bodies())
    {
        body->constraints_.emplace_back(c);
        body->wake();
    }

    return c;
}

template <std::derived_from<ForceField> T, class... Args>
T* World::makeForceField(Args&&... args)
{
    T* f = static_cast<T*>(&*forces_.emplace_back<T>(std::forward<Args>(args)...));
    f->onConstruction(*this);

    if (f->domain().type == ForceField::DomainType::global)
        for (Body& body : bodies())
            body.wake();
    else
        colliderTree_.forEachIn(f->domain().region, [](ColliderTree::iterator it) {
            (*it)->body()->wake();
        });

    return f;
}

template <std::derived_from<PhysicsObject> T, class A, class... Args>
UniquePtr<T> World::makeObject(A& alloc, Args&&... args)
{
    auto obj = makeUnique<T>(alloc, std::forward<Args>(args)...);
    obj->onConstruction(*this);
    return obj;
}

template <Callable<void(Body*)> F>
void World::forEachIn(BoundingBox box, const F& func)
{
    colliderTree_.forEachIn(box, [&](ColliderTree::iterator it) {
        func((*it)->body());
    });
}

template <Callable<void(Body*)> F>
void World::forEachAt(Vec2 point, const F& func)
{
    colliderTree_.forEachAt(point, [&](ColliderTree::iterator it) {
        func((*it)->body());
    });
}

} // namespace simu
