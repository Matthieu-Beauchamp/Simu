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

namespace simu
{

bool Body::interactsAsStructural() const
{
    bool isStruct = true;
    for (Constraint* constraint : constraints_)
        isStruct = isStruct && constraint->bodies().isBodyStructural(this);
    for (ContactConstraint* constraint : contacts_)
        isStruct = isStruct && constraint->bodies().isBodyStructural(this);

    return isStruct;
}

} // namespace simu
