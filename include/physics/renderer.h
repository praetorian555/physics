#pragma once

#include <memory>

#if PHYSICS_RENDER

#include "rndr/render/graphicscontext.h"

namespace Physics
{

class Renderer
{
public:
    Renderer(rndr::GraphicsContext* Context, int WindowWidth, int WindowHeight);
    ~Renderer() = default;

    void Resize(int Width, int Height);

private:
    rndr::GraphicsContext* m_GraphicsContext;
    int m_Width = 0;
    int m_Height = 0;
};

}  // namespace Physics

#endif
