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

#include "Simu/physics/Constraint.hpp"
#include "Simu/physics/World.hpp"

namespace simu
{

template <ConstraintFunction F, ConstraintSolver S>
void ConstraintImplementation<F, S>::onConstruction(World& world)
{
    if (disableContacts_)
    {
        auto b = bodies().bodies();
        world.declareContactConflict(Bodies{b[0], b[1]});
    }
};

template <ConstraintFunction F, ConstraintSolver S>
void ConstraintImplementation<F, S>::onDestruction(World& world)
{
    if (disableContacts_)
    {
        auto b = bodies().bodies();
        world.removeContactConflict(Bodies{b[0], b[1]});
    }
};


} // namespace simu
