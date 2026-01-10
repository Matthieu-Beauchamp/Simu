////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2026 Matthieu Beauchamp-Boulay
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
#include <cstdint>


// TODO: To handle Newton's cradle problem:
// Current issue
// Let the collisions between 3 objects: ->A-B-C
// Where A is the only one with velocity.
// Suppose a restitution coefficient of 1 (perfectly elastic)
// Suppose A has velocity of 1
// Step 1:
//  Initialisation:
//      A-B: J M^-1 J^T lambda = - (Jv + eJv_0)
//                             = 2
//      B-C: J M^-1 J^T lambda = 0 + e*0
//  Solve:
//      A -> velocity = 0; B -> velocity = 1
//      B-C: J M^-1 J^T lambda = Jv + e*0 = 1
//  *** restitution between B-C is not accounted for,
//      B -> velocity = 0.5; C -> velocity = 0.5
//
// Brian Mirtich addresses this by separating collisions between
//  resting contacts and collisions (dynamic)
// The criteria is that the relative velocity v_n < sqrt(epsilon * 2g)
//
// See the discussion here: https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=1584
// Jan Bender's paper here: https://animation.rwth-aachen.de/media/papers/2006-CASA-ConstraintBasedCollisions.pdf
//
// The idea would be to separate solving dynamic and resting contacts.
// Dynamic contacts should probably not use warmstarting,
//  instead try to one shot the collision and mark it as done.
//
// Jan Bender's algo:
// Solve all dynamic collisions sequentially until they reach their desired relative
//  velocity. That means collision B-C would initially be tagged as resting contact,
//  but then added to the dynamic collisions and the proper restitution would
//  be computed and used.
//
// Suppose instead that we have ->A-B-C<-
// where both A-B and B-C are dynamic collisions with velocity 1 and 2, resp.
//
// Our algo:
// Step 1:
//  Initialisation:
//      A-B: J M^-1 J^T lambda = - (Jv + eJv_0)
//                             = 2
//      B-C: J M^-1 J^T lambda = - (Jv + eJv_0)
//                             = 4
//  Solve iteration 1:
//      A -> velocity = 0; B -> velocity = 1
//
//      B-C: J M^-1 J^T lambda = Jv + e*2 = 5
//      B -> velocity = 1 - 2.5 = -1.5; C -> velocity = -2 + 2.5 = 0.5
//  Solve iteration 2:
//      A-B: J M^-1 J^T lambda = - (Jv + eJv_0) = - (-1.5 - 1)
//                             = 2.5
//      A -> velocity = - 1.25;
//      B -> velocity = -1.5 + 1.25 = -0.25
//
//      B-C: J M^-1 J^T lambda = - (Jv + eJv_0) = - (0.75 - 2)
//                             = 1.25
//      B -> velocity = -0.25 - 0.625 = -0.875
//      C -> velocity = 0.5 + 0.625 = 1.125
//
//  Solve iteration 3:
//      A-B: J M^-1 J^T lambda = - (Jv + eJv_0) = - (1.25-0.875 - 1) = 0.625
//      A -> velocity = - 1.5625;
//      B -> velocity = -0.875 + 0.3125 ~= -0.5625
//
//      B-C: J M^-1 J^T lambda = - (Jv + eJv_0) = - (1.6875 - 2) ~= 0.3
//      B -> velocity = -0.5625 - 0.15 ~= -0.7125
//      C -> velocity = 1.125 + 0.15 = 1.275
//
//  ... unclear if this actually converges to A=-2 and C = 1
//
// Processing dynamic collisions one shot is most likely the simplest way to achieve
//  realistic dynamic results: A=1 B=0 C=-2
//  Process A-B fully: A=0; B=1; mark done
//  Process B-C fully using current vel for restitution: Jv = 3, eJv = 3 -> 6
//                                          => B=-2; C = 1; mark done
//  Loop to see if dynamic collisions remain, see A-B => A=-2 B=0; mark done.
//  No more collisions to process.
//
// Order may not be important, simply looping over contacts could be sufficient.
// Resting contacts will be unstable if processed this way.
// Solve contacts normally after processing dynamic collisions.
//

namespace simu
{

// TODO: Implement and add to settings
enum class ContactSolverStrategy
{
    /// Sequential Impulse with optional warmstarting.
    /// This is the algorithm described by Erin Catto
    SequentialImpulse,

    /// Dynamic collisions are processed fully one at a time until there are none left (or max iterations)
    /// Resting contacts (below a velocity threshold) use sequential impulse
    PairwiseNewton
};

} // namespace simu
