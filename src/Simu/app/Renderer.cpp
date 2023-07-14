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

#include <numbers>

#include "glbinding/gl33core/gl.h"

#include "Simu/app/Renderer.hpp"
#include "Simu/math/Polygon.hpp"

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
    SIMU_ASSERT(contour == Contour::outside, "Not implemented");

    Vertices outer{vertices.begin(), vertices.end()};

    Uint32 i     = 0;
    auto   edges = edgesOf(vertices);
    for (auto e1 = edges.begin(); e1 != edges.end(); ++e1)
    {
        auto e2 = edges.next(e1);
        outer[i++]
            += (e1->normalizedNormal() + e2->normalizedNormal()) * contourWidth;
    }

    const Vertices& cOuter = outer;
    drawPolygon(
        center,
        makeView(cOuter.data(), cOuter.data() + cOuter.size()),
        contourColor
    );

    drawPolygon(center, vertices, fillColor);
}


void Renderer::drawTriangle(Vec2 A, Vec2 B, Vec2 C, Rgba color)
{
    Polygon triangle{A, B, C};
    drawPolygon(triangle.properties().centroid, triangle.vertexView(), color);
}

void Renderer::drawLine(Vec2 A, Vec2 B, Rgba color, float width, LineTip tip)
{
    SIMU_ASSERT(any(A != B), "A and B must not be the same point");

    Vertices v;
    Vec2     n = width * normalized(perp(B - A));

    switch (tip)
    {
        case LineTip::rounded:
        case LineTip::triangle: SIMU_ASSERT(false, "Not implemented"); break;
        default:
            v.emplace_back(A + n / 2);
            v.emplace_back(B + n / 2);
            v.emplace_back(B - n / 2);
            v.emplace_back(A - n / 2);
            break;
    }

    Polygon line{v.begin(), v.end()};
    drawPolygon(line.properties().centroid, line.vertexView(), color);
}

void Renderer::drawPoint(Vec2 P, Rgba color, float radius, Uint8 precision)
{
    SIMU_ASSERT(precision >= 3, "");

    Vertices        v;
    constexpr float pi = std::numbers::pi_v<float>;
    for (Uint8 i = 0; i < precision; ++i)
    {
        float theta = 2.f * pi * i / precision;
        v.emplace_back(P + radius* Vertex{std::cos(theta), std::sin(theta)});
    }

    const Vertices& cv = v;
    drawPolygon(P, makeView(cv.data(), cv.data() + cv.size()), color);
}


////////////////////////////////////////////////////////////
// OpenGl Renderer
////////////////////////////////////////////////////////////

gl::GLuint compileShader(gl::GLenum shaderType, const char* src)
{
    // https://www.khronos.org/opengl/wiki/Shader_Compilation

    gl::GLuint shader = gl::glCreateShader(shaderType);

    // Get strings for glShaderSource.
    gl::GLint len = std::strlen(src);
    gl::glShaderSource(shader, 1, &src, &len);

    gl::glCompileShader(shader);

    gl::GLint isCompiled = 0;
    gl::glGetShaderiv(shader, gl::GL_COMPILE_STATUS, &isCompiled);
    if (isCompiled == gl::GL_FALSE)
    {
        gl::GLint maxLength = 0;
        gl::glGetShaderiv(shader, gl::GL_INFO_LOG_LENGTH, &maxLength);

        // The maxLength includes the NULL character
        std::string errorLog{};
        errorLog.resize(maxLength);
        gl::glGetShaderInfoLog(shader, maxLength, &maxLength, &errorLog[0]);

        // Provide the infolog in whatever manor you deem best.
        // Exit with failure.
        gl::glDeleteShader(shader); // Don't leak the shader.

        throw simu::Exception("GL shader compilation failed:\n" + errorLog);
    }
    else
    {
        return shader;
    }
}

gl::GLuint compileShaderProgram(const char* vSrc, const char* fSrc)
{
    gl::GLuint vertexShader   = compileShader(gl::GL_VERTEX_SHADER, vSrc);
    gl::GLuint fragmentShader = compileShader(gl::GL_FRAGMENT_SHADER, fSrc);

    // https://www.khronos.org/opengl/wiki/Shader_Compilation

    // Vertex and fragment shaders are successfully compiled.
    // Now time to link them together into a program.
    // Get a program object.
    gl::GLuint program = gl::glCreateProgram();

    // Attach our shaders to our program
    gl::glAttachShader(program, vertexShader);
    gl::glAttachShader(program, fragmentShader);

    // Link our program
    gl::glLinkProgram(program);

    // Note the different functions here: glGetProgram* instead of glGetShader*.
    gl::GLint isLinked = 0;
    gl::glGetProgramiv(program, gl::GL_LINK_STATUS, (int*)&isLinked);
    if (isLinked == gl::GL_FALSE)
    {
        gl::GLint maxLength = 0;
        gl::glGetProgramiv(program, gl::GL_INFO_LOG_LENGTH, &maxLength);

        // The maxLength includes the NULL character
        std::string errorLog{};
        errorLog.resize(maxLength);
        gl::glGetProgramInfoLog(program, maxLength, &maxLength, &errorLog[0]);

        // We don't need the program anymore.
        gl::glDeleteProgram(program);
        // Don't leak shaders either.
        gl::glDeleteShader(vertexShader);
        gl::glDeleteShader(fragmentShader);

        throw simu::Exception("GL shader linking failed:\n" + errorLog);
    }

    // Always detach shaders after a successful link.
    gl::glDetachShader(program, vertexShader);
    gl::glDetachShader(program, fragmentShader);

    return program;
}

OpenGlRenderer::OpenGlRenderer()
{
    programId_ = compileShaderProgram(vertexShaderSrc_, fragmentShaderSrc_);
    cameraTransformBinding_
        = gl::glGetUniformLocation(programId_, "cameraTransform");

    gl::glGenVertexArrays(1, &vao_);
    gl::glGenBuffers(1, &vbo_);
    gl::glGenBuffers(1, &ebo_);

    gl::glBindVertexArray(vao_);
    gl::glBindBuffer(gl::GL_ARRAY_BUFFER, vbo_);
    gl::glBindBuffer(gl::GL_ELEMENT_ARRAY_BUFFER, ebo_);

    gl::glVertexAttribPointer(
        0,
        2,
        gl::GL_FLOAT,
        gl::GL_FALSE,
        sizeof(OpenGlRenderer::Vertex),
        (void*)offsetof(OpenGlRenderer::Vertex, pos)
    );
    gl::glEnableVertexAttribArray(0);

    gl::glVertexAttribPointer(
        1,
        4,
        gl::GL_UNSIGNED_BYTE,
        gl::GL_TRUE,
        sizeof(OpenGlRenderer::Vertex),
        (void*)offsetof(OpenGlRenderer::Vertex, color)
    );
    gl::glEnableVertexAttribArray(1);

    gl::glBindVertexArray(0);
}

OpenGlRenderer::~OpenGlRenderer()
{
    gl::glDeleteProgram(programId_);

    gl::glDeleteVertexArrays(1, &vao_);
    gl::glDeleteBuffers(1, &vbo_);
    gl::glDeleteBuffers(1, &ebo_);
}

void OpenGlRenderer::drawPolygon(Vec2 center, Poly vertices, Rgba color)
{
    Uint16 centerIndex = vertices_.size();

    vertices_.emplace_back(center, color);
    for (Vec2 v : vertices)
        vertices_.emplace_back(v, color);

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

    if (vertices_.size() >= maxVertices_)
        flush();
}

void OpenGlRenderer::flush()
{
    gl::glUseProgram(programId_);

    const Mat3& rowMajorTransform = cameraTransform();
    gl::glUniformMatrix3fv(
        cameraTransformBinding_,
        1,
        gl::GL_TRUE,
        rowMajorTransform.data
    );

    // https://learnopengl.com/Getting-started/Hello-Triangle

    gl::glBindVertexArray(vao_);

    // vbo_ is already rebound?
    gl::glBindBuffer(gl::GL_ARRAY_BUFFER, vbo_);
    gl::glBufferData(
        gl::GL_ARRAY_BUFFER,
        vertices_.size() * sizeof(OpenGlRenderer::Vertex),
        vertices_.data(),
        gl::GL_STREAM_DRAW
    );

    // ebo_ is already rebound?
    gl::glBindBuffer(gl::GL_ELEMENT_ARRAY_BUFFER, ebo_);
    gl::glBufferData(
        gl::GL_ELEMENT_ARRAY_BUFFER,
        indices_.size() * sizeof(Uint16),
        indices_.data(),
        gl::GL_STREAM_DRAW
    );

    gl::glDrawElements(gl::GL_TRIANGLES, indices_.size(), gl::GL_UNSIGNED_SHORT, 0);

    gl::glBindVertexArray(0);

    vertices_.clear();
    indices_.clear();
}

void OpenGlRenderer::fillScreen(Rgba color) {
    FloatRgba col = static_cast<FloatRgba>(color) / 255.f;
    gl::glClearColor(col[0], col[1], col[2], col[3]);
    gl::glClear(gl::GL_COLOR_BUFFER_BIT);
}

void OpenGlRenderer::setViewport(Vec2i lowerLeft, Vec2i dim)
{
    gl::glViewport(lowerLeft[0], lowerLeft[1], dim[0], dim[1]);
}

const char* const OpenGlRenderer::vertexShaderSrc_ = R"(
#version 330 core
layout (location = 0) in vec2 pos;
layout (location = 1) in vec4 col;

uniform mat3 cameraTransform;

out vec4 vCol;

void main()
{
    gl_Position = vec4(pos, 0.0, 1.0); 
    vCol = col;
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
