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

#include <vector>

#include "Simu/config.hpp"
#include "Simu/math/Geometry.hpp"
#include "Simu/utility/View.hpp"
#include "Simu/utility/Memory.hpp"

namespace simu
{

typedef Vector<Uint8, 4> Rgba;

class Renderer
{
public:

    Renderer()          = default;
    virtual ~Renderer() = default;

    // vertices are assumed to be positively oriented.
    // All vertices must be seen from center, ie a line from center to
    //      any of the vertices is fully contained in the polygon.
    typedef ViewType<const Vec2*> Poly;
    // TODO: Make overloads that take ViewType<Vec2*>, convert begin and end.

    virtual void drawPolygon(Vec2 center, Poly vertices, Rgba color) = 0;
    virtual void flush()                                             = 0;
    virtual void fillScreen(Rgba color)                              = 0;
    virtual void setViewport(Vec2i lowerLeft, Vec2i dim)             = 0;

    void        setCameraTransform(const Mat3& cameraTransform);
    const Mat3& cameraTransform() const;

    enum class Contour
    {
        outside,
        inside,
        over
    };

    void drawContouredPolygon(
        Vec2    center,
        Poly    vertices,
        Rgba    fillColor,
        Rgba    contourColor,
        float   contourWidth,
        Contour contour = Contour::outside
    );


    void drawTriangle(Vec2 A, Vec2 B, Vec2 C, Rgba color);

    enum class LineTip
    {
        square,
        triangle,
        rounded
    };

    void
    drawLine(Vec2 A, Vec2 B, Rgba color, float width, LineTip tip = LineTip::square);

    void drawPoint(Vec2 P, Rgba color, float radius, Uint8 precision = 4);

protected:

    typedef FreeListAllocator<Vec2> Alloc;

    Alloc alloc;

private:

    typedef std::vector<Vec2, Alloc> Vertices;

    Vertices v_;

    Mat3 cameraTransform_;
};


class OpenGlRenderer : public Renderer
{
public:

    OpenGlRenderer();
    ~OpenGlRenderer() override;

    void drawPolygon(Vec2 center, Poly vertices, Rgba color) override;
    void flush() override;
    void fillScreen(Rgba color) override;
    void setViewport(Vec2i lowerLeft, Vec2i dim) override;

private:

    typedef Vector<float, 4> FloatRgba;


    struct Vertex
    {
        Vertex(Vec2 pos, Rgba color) : pos{pos}, color{color} {}

        Vec2 pos;
        Rgba color;
    };


    std::vector<Vertex, typename Alloc::rebind<Vertex>::other> vertices_{};
    std::vector<Uint16, typename Alloc::rebind<Uint16>::other> indices_{};
    static const std::size_t maxVertices_ = 1024;

    static const char* const vertexShaderSrc_;
    static const char* const fragmentShaderSrc_;

    unsigned int programId_;
    unsigned int cameraTransformBinding_;

    unsigned int vao_;
    unsigned int vbo_;
    unsigned int ebo_;
};

} // namespace simu
