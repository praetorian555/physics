#include "physics/renderer.h"

#if PHYSICS_RENDER

physics::Renderer::Renderer(rndr::GraphicsContext* Context, int WindowWidth, int WindowHeight)
    : m_GraphicsContext(Context), m_Width(WindowWidth), m_Height(WindowHeight)
{
}

void physics::Renderer::Resize(int Width, int Height)
{
    m_Width = Width;
    m_Height = Height;
}

#endif
