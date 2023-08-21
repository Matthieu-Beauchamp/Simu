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

#include <numbers>

#include "Simu/config.hpp"
#include "Simu/utility/Algo.hpp"

namespace simu
{

////////////////////////////////////////////////////////////
/// \brief See https://youtu.be/UUt4Lko2wFI for visualizations
///
///
////////////////////////////////////////////////////////////
class ConstraintSoftness
{
public:

    ConstraintSoftness() { set(Feedbacks{}); }

    ////////////////////////////////////////////////////////////
    /// \brief Used by constraints.
    ///
    ////////////////////////////////////////////////////////////
    struct Feedbacks
    {
        float beta;  // error feedback
        float gamma; // force feedback
    };

    ////////////////////////////////////////////////////////////
    /// \brief Defines the softness of the constraint like a damped harmonic oscillator
    ///
    /// https://en.wikipedia.org/wiki/Harmonic_oscillator#Damped_harmonic_oscillator
    ////////////////////////////////////////////////////////////
    struct Harmonic
    {
        float dampingRatio;     // zeta
        float angularFrequency; // omega
    };

    ////////////////////////////////////////////////////////////
    /// \brief Defines the softness of the constraint by the back and forth frequency and half life of the error.
    ///
    /// The constraint error is split in half every halfLife units of time.
    /// The constraint error oscillates at frequency Hz.
    ////////////////////////////////////////////////////////////
    struct HalfLife
    {
        float frequency;
        float halfLife;
    };

    ////////////////////////////////////////////////////////////
    /// \brief Produces no oscillation, splits the constraint error by half every halfLife units of time.
    ///
    ////////////////////////////////////////////////////////////
    struct Decay
    {
        float halfLife;
    };

    ////////////////////////////////////////////////////////////
    /// For a scalar constraint (dimension = 1), the mass parameter is
    ///     (J * M^-1 * J^T)^1. Otherwise take the corresponding entry in the
    ///     diagonal of that matrix.
    ////////////////////////////////////////////////////////////
    Feedbacks getFeedbacks(float mass, float dt) const
    {
        // TODO: Since we don't have any getters, convert on set().
        switch (param_)
        {
            case Parametrization::feedbacks: return feedbacks_;
            case Parametrization::harmonic:
                return feedbacksFromHarmonic(harmonic_, mass, dt);
            case Parametrization::halfLife:
                return feedbacksFromHalfLife(halfLife_, mass, dt);
            case Parametrization::decay:
                return feedbacksFromDecay(decay_, mass, dt);
            default: return Feedbacks{};
        }
    }

    void set(const Feedbacks& feedbacks)
    {
        param_     = Parametrization::feedbacks;
        feedbacks_ = feedbacks;
    }

    void set(const Harmonic& harmonic)
    {
        param_    = Parametrization::harmonic;
        harmonic_ = harmonic;
    }

    void set(const HalfLife& halfLife)
    {
        param_    = Parametrization::halfLife;
        halfLife_ = halfLife;
    }

    void set(const Decay& decay)
    {
        param_ = Parametrization::decay;
        decay_ = decay;
    }

private:

    enum class Parametrization
    {
        feedbacks,
        harmonic,
        halfLife,
        decay
    };

    // internal only
    struct Spring
    {
        float damping;
        float stiffness;
    };


    static Feedbacks feedbacksFromSpring(const Spring& s, float dt)
    {
        Feedbacks f{};

        float denum = s.damping + dt * s.stiffness;
        float inv   = 1.f / denum;

        f.beta  = dt * s.stiffness * inv;
        f.gamma = inv;
        return f;
    }

    static Feedbacks
    feedbacksFromHarmonic(const Harmonic& harm, float mass, float dt)
    {
        Spring s{};
        s.damping   = 2 * mass * harm.dampingRatio * harm.angularFrequency;
        s.stiffness = mass * squared(harm.angularFrequency);

        return feedbacksFromSpring(s, dt);
    }

    static Feedbacks
    feedbacksFromHalfLife(const HalfLife& halfLife, float mass, float dt)
    {
        Harmonic harm{};
        harm.angularFrequency = 2 * std::numbers::pi_v<float> * halfLife.frequency;
        // ln(2) == -ln(0.5)
        harm.dampingRatio = std::numbers::ln2_v<float>
                            / (harm.angularFrequency * halfLife.halfLife);

        return feedbacksFromHarmonic(harm, mass, dt);
    }

    static Feedbacks feedbacksFromDecay(const Decay& decay, float mass, float dt)
    {
        Harmonic harm{};
        harm.dampingRatio = 1;

        // ln(2) == -ln(0.5)
        harm.angularFrequency = std::numbers::ln2_v<float> / decay.halfLife;

        return feedbacksFromHarmonic(harm, mass, dt);
    }


    Parametrization param_;

    union
    {
        Feedbacks feedbacks_;
        Spring    spring_;
        Harmonic  harmonic_;

        HalfLife halfLife_;
        Decay    decay_;
    };
};


} // namespace simu
