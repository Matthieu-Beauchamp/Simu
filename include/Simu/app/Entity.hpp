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

#include "Simu/physics/World.hpp"

#include "Simu/app/Drawable.hpp"


namespace simu
{

class Entity : public Drawable
{
public:

    Entity() = default;

    // should kill all the physics objetcs they created
    ~Entity() override = default;

    // create associated physics objects
    virtual void declarePhysics(World& world) = 0;
};


class InvisibleEntity : public Entity
{
public:

    InvisibleEntity()           = default;
    ~InvisibleEntity() override = default;

    void draw(Renderer& /* renderer */) override {}
};


} // namespace simu
