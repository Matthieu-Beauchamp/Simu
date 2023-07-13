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

#include "Simu/app/Renderer.hpp"

namespace simu
{

////////////////////////////////////////////////////////////
// Renderer
////////////////////////////////////////////////////////////

void Renderer::setCameraTransform(const Mat3& cameraTransform)
{
    cameraTransform_ = cameraTransform;
}

const Mat3& Renderer::cameraTransform() const { return cameraTransform_; }


void Renderer::drawContouredPolygon(
    Vec2    center,
    Poly    vertices,
    Rgba    fillColor,
    Rgba    contourColor,
    float   contourWidth,
    Contour contour
)
{
}


void Renderer::drawTriangle(Vec2 A, Vec2 B, Vec2 C, Rgba color) {}

void Renderer::drawLine(
    Vec2    A,
    Vec2    B,
    Rgba    color,
    float   width,
    LineTip tip
)
{
}

void Renderer::drawPoint(Vec2 P, Rgba color, float radius, Uint8 precision )
{
}


////////////////////////////////////////////////////////////
// OpenGl Renderer
////////////////////////////////////////////////////////////

OpenGlRenderer::OpenGlRenderer() {}

void OpenGlRenderer::drawPolygon(Vec2 center, Poly vertices, Rgba color) 
{
    Uint16 centerIndex = vertices_.size();

    vertices_.emplace_back(center);
    for (Vec2 v : vertices)
        vertices_.emplace_back(v);

    Uint16 end = vertices_.size();

    indices_.emplace_back(centerIndex);
    indices_.emplace_back(end - 1);
    indices_.emplace_back(centerIndex + 1);

    Uint16 index = centerIndex + 2;
    while (index < end)
    {
        indices_.emplace_back(centerIndex);
        indices_.emplace_back(index - 1);
        indices_.emplace_back(index++);
    }

    if (index >= maxVertices_)
        flush();
}

void OpenGlRenderer::flush(){}

const char* const OpenGlRenderer::vertexShaderSrc_ = R"(
#version 330 core
layout (location = 0) in vec2 pos;
layout (location = 1) in uint col;

uniform mat3 cameraTransform;

out vec4 vCol;

void main()
{
    gl_Position = vec4(pos, 0.0, 1.0); 

    vCol.r = col & 0xFF000000;
    vCol.g = col & 0x00FF0000;
    vCol.b = col & 0x0000FF00;
    vCol.a = col & 0x000000FF;
    vCol = vCol / 255.0;
}

)";

const char* const OpenGlRenderer::fragmentShaderSrc_ = R"(
#version 330 core
in vec4 vCol;
out vec4 fCol;

void main()
{
    fCol = vCol;
} 

)";


} // namespace simu
