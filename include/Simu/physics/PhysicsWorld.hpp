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
#include "Simu/physics/RTree.hpp"
#include "Simu/physics/PhysicsBody.hpp"
#include "Simu/physics/ForceField.hpp"
#include "Simu/physics/Range.hpp"

namespace details
{
// https://youngforest.github.io/2020/05/27/best-implement-to-use-pair-as-key-to-std-unordered-map-in-C/
// from boost (functional/hash):
// see http://www.boost.org/doc/libs/1_35_0/doc/html/hash/combine.html template
template <typename T>
inline void hash_combine(std::size_t& seed, const T& val)
{
    seed ^= std::hash<T>()(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

// auxiliary generic functions to create a hash value using a seed
template <typename T>
inline void hash_val(std::size_t& seed, const T& val)
{
    hash_combine(seed, val);
}
template <typename T, typename... Types>
inline void hash_val(std::size_t& seed, const T& val, const Types&... args)
{
    hash_combine(seed, val);
    hash_val(seed, args...);
}

template <typename... Types>
inline std::size_t hash_val(const Types&... args)
{
    std::size_t seed = 0;
    hash_val(seed, args...);
    return seed;
}

} // namespace details

namespace std
{

template <>
struct hash<simu::Bodies<2>>
{
    size_t operator()(const simu::Bodies<2>& bodies) const
    {
        return ::details::hash_val(bodies[0], bodies[1]);
    }
};

} // namespace std

#include <list>
#include <unordered_map>


namespace simu
{

class Constraint;

class PhysicsWorld
{
    typedef RTree<std::unique_ptr<PhysicsBody>> BodyTree;

public:

    typedef PhysicsBody*       BodyPtr;
    typedef const PhysicsBody* ConstBodyPtr;

    typedef Constraint*       ConstraintPtr;
    typedef const Constraint* ConstConstraintPtr;

    typedef ForceField*       ForceFieldPtr;
    typedef const ForceField* ConstForceFieldPtr;

    PhysicsWorld() = default;

    template <std::derived_from<PhysicsBody> T = PhysicsBody, class... Args>
    T* makeBody(Args&&... args)
    {
        std::unique_ptr<T> body = makeObject<T>(std::forward<Args>(args)...);

        BoundingBox bounds = body->collider().boundingBox();
        return static_cast<T*>(bodies_.emplace(bounds, std::move(body))->get());
    }

    template <std::derived_from<Constraint> T, class... Args>
    T* makeConstraint(Args&&... args)
    {
        return static_cast<T*>(
            constraints_
                .emplace_back(makeObject<T>(std::forward<Args>(args)...))
                .get()
        );
    }

    template <std::derived_from<ForceField> T, class... Args>
    T* makeForceField(Args&&... args)
    {
        return static_cast<T*>(
            forces_.emplace_back(makeObject<T>(std::forward<Args>(args)...)).get()
        );
    }

    void step(float dt);


    auto bodies()
    {
        return makeRange(bodies_.begin(), bodies_.end(), BypassSmartPointer{});
    }
    auto bodies() const
    {
        return makeRange(bodies_.begin(), bodies_.end(), BypassSmartPointer{});
    }

    void declareContactConflict(const Bodies<2>& bodies);

    void removeContactConflict(const Bodies<2>& bodies);

private:

    auto forceFields()
    {
        return makeRange(forces_.begin(), forces_.end(), BypassSmartPointer{});
    }
    auto forceFields() const
    {
        return makeRange(forces_.begin(), forces_.end(), BypassSmartPointer{});
    }

    auto constraints()
    {
        return makeRange(
            constraints_.begin(),
            constraints_.end(),
            BypassSmartPointer{}
        );
    }
    auto constraints() const
    {
        return makeRange(
            constraints_.begin(),
            constraints_.end(),
            BypassSmartPointer{}
        );
    }

    template <std::derived_from<PhysicsObject> T, class... Args>
    std::unique_ptr<T> makeObject(Args&&... args)
    {
        auto obj = std::make_unique<T>(std::forward<Args>(args)...);
        obj->onConstruction(*this);
        return obj;
    }


    void applyForces(float dt);

    void detectContacts();

    void cleanup();

    void applyConstraints(float dt);

    void updateBodies(float dt);

    BodyTree                               bodies_{};
    std::list<std::unique_ptr<Constraint>> constraints_{};
    std::list<std::unique_ptr<ForceField>> forces_{};


    struct ContactStatus
    {
        Int32         nConflictingConstraints = 0;
        ConstraintPtr existingContact         = nullptr;
    };

    std::unordered_map<Bodies<2>, ContactStatus> contacts_;
    std::unordered_map<Bodies<2>, ContactStatus>::iterator
    inContacts(Bodies<2> bodies)
    {
        auto asIs = contacts_.find(bodies);
        return (asIs != contacts_.end())
                   ? asIs
                   : contacts_.find(Bodies<2>{bodies[1], bodies[0]});
    }

    // TODO:
    // ConstraintSolver solver_;
    // solver settings
};


} // namespace simu
