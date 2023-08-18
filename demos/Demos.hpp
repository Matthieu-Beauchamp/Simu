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

#include "Simu/app.hpp"


class BoxStacks : public simu::Scene
{
public:

    BoxStacks();
    void init(simu::Renderer& renderer) override;
    void doGui() override;

private:

    int nStacks_ = 10;
    int height_  = 10;
};


class NewtonPendulum : public simu::Scene
{
public:

    NewtonPendulum();
    void init(simu::Renderer& renderer) override;
    void doGui() override;

private:

    // n above 10 is not very stable, increasing velocity iterations does not help.
    //  (issue seems to be with bouncing)
    int n_ = 5;

    float circleRadius_ = 1.f;
    float stringLength_ = 7.f;

    int circlePrecision_ = 48;

    int initialPush_ = 1;
};

class Pyramid : public simu::Scene
{
public:

    Pyramid();
    void init(simu::Renderer& renderer) override;
    void doGui() override;

private:

    static constexpr float w = 2.f;
    static constexpr float h = 2.f;

    int   height_  = 60;
    float spacing_ = 0.5f;
};


class Tower : public simu::Scene
{
public:

    Tower();
    void init(simu::Renderer& renderer) override;
    void doGui() override;

private:

    static constexpr float w         = 5.f;
    static constexpr float thickness = 1.f;

    void makeSlab(simu::Vec2 pos, bool vertical);

    int height_ = 20;
};


class Tumbler : public simu::Scene
{
public:


    Tumbler();
    void init(simu::Renderer& renderer) override;
    void doGui() override;

    void postStep(float) override;

private:

    int maxCount_ = 1500;

    int count_ = 0;
};
