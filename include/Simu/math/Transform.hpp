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

#include <cmath>
#include <numbers>

#include "Simu/config.hpp"
#include "Simu/math/Matrix.hpp"

namespace simu
{

class Rotation
{
public:

    explicit Rotation(float theta) { set(theta); }
    explicit operator Mat3() const
    {
        // clang-format off
        return Mat3{
            cosine, -sine,   0.f, 
            sine,    cosine, 0.f, 
            0.f,     0.f,    1.f
        };
        // clang-format on
    }

    Rotation inverse() const { return Rotation{-theta(), cosine, -sine}; }

    float theta() const { return theta_; }
    void  set(float theta)
    {
        // makes Rotation Constraint bug out 
        // constexpr float fullCircle = 2.f * std::numbers::pi_v<float>;

        // float nCircles = theta / fullCircle;
        // theta -= std::floor(nCircles) * fullCircle;

        theta_ = theta;
        cosine = std::cos(theta);
        sine   = std::sin(theta);
    }

    Vec2 operator*(const Vec2& v) const
    {
        return Vec2{cosine * v[0] - sine * v[1], sine * v[0] + cosine * v[1]};
    }

    Rotation& operator*=(const Rotation& other)
    {
        set(theta() + other.theta());
        return *this;

        // numerical error accumulates,
        //  in the Pyramid demo, the pyramid seems to breathe.
        //
        // theta_ += other.theta();
        // float c = cosine * other.cosine - sine * other.sine;
        // sine    = sine * other.cosine + cosine * other.sine;
        // cosine  = c;
        // return *this;
    }

    Rotation operator*(const Rotation& other) const
    {
        Rotation cpy{*this};
        cpy *= other;
        return cpy;
    }

private:

    Rotation(float theta, float c, float s) : theta_{theta}, cosine{c}, sine{s}
    {
    }

    float theta_;

    float cosine;
    float sine;
};


class Translation
{
public:

    explicit Translation(Vec2 offset) : offset_{offset} {}
    explicit operator Mat3() const
    {
        // clang-format off
        return Mat3{
            1, 0, offset_[0], 
            0, 1, offset_[1], 
            0, 0, 1
        };
        // clang-format on
    }

    Translation inverse() const { return Translation{-offset()}; }

    Vec2 offset() const { return offset_; }
    void set(Vec2 offset) { offset_ = offset; }

    Vec2 operator*(const Vec2& v) const { return v + offset_; }

    Translation operator*=(const Translation& other)
    {
        offset_ += other.offset();
        return *this;
    }

    Translation operator*(const Translation& other) const
    {
        return Translation{offset() + other.offset()};
    }

private:

    Vec2 offset_;
};


class Transform
{
public:

    explicit Transform() : Transform(Rotation{0.f}, Translation{Vec2{}}) {}

    Transform(Rotation r, Translation t) : r_{r}, t_{t} {}
    explicit operator Mat3() const
    {
        Mat3 T{r_};
        Vec2 trans = t_.offset();
        T(0, 2)    = trans[0];
        T(1, 2)    = trans[1];
        return T;
    }

    inline Transform inverse() const;

    ////////////////////////////////////////////////////////////
    /// \brief No-op transformation
    ///
    ////////////////////////////////////////////////////////////
    static Transform identity() { return Transform{}; }


    ////////////////////////////////////////////////////////////
    /// \brief Rotates a 2D vector by theta radians around the origin {0, 0}
    ///
    ////////////////////////////////////////////////////////////
    static Rotation rotation(float theta) { return Rotation{theta}; }

    ////////////////////////////////////////////////////////////
    /// \brief Translate a 2D vector by offset
    ///
    ////////////////////////////////////////////////////////////
    static Translation translation(Vec2 offset) { return Translation{offset}; }


    ////////////////////////////////////////////////////////////
    /// \brief Rotates around transformOrigin by theta radians and then translates by offset.
    ///
    ////////////////////////////////////////////////////////////
    static inline Transform
    transformAround(float theta, Vec2 offset, Vec2 transformOrigin);

    ////////////////////////////////////////////////////////////
    /// \brief The rotation part of this transform
    ///
    ////////////////////////////////////////////////////////////
    const Rotation& rotation() const { return r_; }
    Rotation&       rotation() { return r_; }

    ////////////////////////////////////////////////////////////
    /// \brief The translation part of this transform
    ///
    ////////////////////////////////////////////////////////////
    const Translation& translation() const { return t_; }
    Translation&       translation() { return t_; }


    Vec2 operator*(const Vec2& v) const { return r_ * v + t_.offset(); }

private:

    Rotation    r_;
    Translation t_;
};


inline Transform operator*(const Rotation& r, const Translation& t)
{
    return Transform(r, Translation(r * t.offset()));
}

inline Transform operator*(const Translation& t, const Rotation& r)
{
    return Transform(r, t);
}


inline Transform operator*(const Transform& T, const Translation& t)
{
    Vec2 offset = T.translation().offset() + T.rotation() * t.offset();
    return Transform{T.rotation(), Translation{offset}};
}

inline Transform operator*(const Transform& T, const Rotation& r)
{
    return Transform(T.rotation() * r, T.translation());
}


inline Transform operator*(const Translation& t, const Transform& T)
{
    return Transform{T.rotation(), T.translation() * t};
}

inline Transform operator*(const Rotation& r, const Transform& T)
{
    return Transform(T.rotation() * r, Translation{r * T.translation().offset()});
}


inline Transform operator*(const Transform& T1, const Transform& T2)
{
    Transform T{T1.rotation() * T2};
    T.translation() *= T1.translation();
    return T;
}


Transform Transform::inverse() const { return r_.inverse() * t_.inverse(); }

Transform
Transform::transformAround(float theta, Vec2 offset, Vec2 transformOrigin)
{
    return translation(offset + transformOrigin) * rotation(theta)
           * translation(-transformOrigin);
}


inline Vec2 operator*(const Mat3& T, Vec2 v)
{
    Vec3 vec{v[0], v[1], 1.f};
    vec = T * vec;
    return Vec2{vec[0], vec[1]};
}

} // namespace simu
