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

#include "Simu/config.hpp"

namespace simu
{

class World;

////////////////////////////////////////////////////////////
/// \brief Base class of physics objects
///
/// The lifetime of physics objects is managed by World.
/// Instead of explicitly removing them from the world, objects must specify
/// themselves when they should be removed.
///
/// Objects will be removed if they are killed with PhysicsObject::kill()
/// or when some condition is satisfied in the virtual PhysicsObject::shouldDie() method.
///
/// The actual removal will be done at the beginning of World::step().
/// Since some objects may want to remove themselves once some other object dies,
/// no object is truly removed until PhysicsObject::isDead() is called on all objects of the world.
/// This means that objects can query other objects in their shouldDie() method without worrying about accessing released memory.
/// The shouldDie() method can also be used to update references to dying objects (which is why it is not const).
///
/// Some objects may want to spawn objects when they are created and kill them when they are destroyed, this can be done with
/// PhysicsObject::onConstruction(world) and PhysicsObject::onDestruction(world).
/// Again, onDestruction will be called for all objects that will be destroyed before
/// any object is truly removed and their memory released.
///
////////////////////////////////////////////////////////////
class PhysicsObject
{
public:

    virtual ~PhysicsObject() = default;

    bool isDead() { return killed_ || shouldDie(); }
    void kill()
    {
        killed_ = true;
        onKill();
    }

protected:

    friend World;
    virtual void onConstruction(World& /* world */){};
    virtual void onDestruction(World& /* world */){};
    virtual void onKill(){};

    virtual bool shouldDie() { return false; }

private:

    bool killed_ = false;
};

} // namespace simu
