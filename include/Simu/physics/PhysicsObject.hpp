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

    ////////////////////////////////////////////////////////////
    /// \brief Called when a World creates this object
    /// 
    /// Subclasses should always call Base::onConstruction
    /// 
    ////////////////////////////////////////////////////////////
    virtual void onConstruction(World& /* world */){};

    ////////////////////////////////////////////////////////////
    /// \brief Called when a World destroys this object
    /// 
    /// Subclasses should always call Base::onDestruction
    ///  
    /// \see onKill 
    ////////////////////////////////////////////////////////////
    virtual void onDestruction(World& /* world */){};
    
    ////////////////////////////////////////////////////////////
    /// \brief Called when this object is requested to die (by a call to kill()) 
    /// 
    /// Subclasses should always call Base::onKill
    /// 
    /// This is different from onDestruction since this object is not destroyed
    /// immediately after this call. It will be destroyed the next time 
    /// World::step is called on the World owning this object.
    /// 
    /// This method should be used to kill other objects linked to this one.
    /// For example a composed Body may create other objects with onConstruction,
    /// keep a reference to them and kill them when it is killed with onKill.
    /// 
    ////////////////////////////////////////////////////////////
    virtual void onKill(){};

    ////////////////////////////////////////////////////////////
    /// \brief Offers an alternative death condition than this->kill().
    /// 
    /// Subclasses should always consider if Base::shouldDie() is true.
    /// 
    /// Example: a Constraint that is broken if the force it applies exceeds 
    ///     a treshold can implement this death condition in shouldDie.
    /// 
    ////////////////////////////////////////////////////////////
    virtual bool shouldDie() { return false; }

    ////////////////////////////////////////////////////////////
    /// \brief Called at the start of World::step, after dead objects are removed.
    /// 
    /// Subclasses should always call Base::preStep().
    /// 
    /// This is called for Body, Constraint and then for ForceField.
    /// 
    ////////////////////////////////////////////////////////////
    virtual void preStep() {}

    ////////////////////////////////////////////////////////////
    /// \brief Called at the end of World::step.
    /// 
    /// Subclasses should always call Base::postStep().
    /// 
    /// This is called for Body, Constraint and then for ForceField.
    /// 
    ////////////////////////////////////////////////////////////
    virtual void postStep() {}


private:

    bool killed_ = false;
};

} // namespace simu
