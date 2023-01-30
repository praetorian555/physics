
#include "rndr/rndr.h"

#include "physics/renderer.h"

class Fireworks
{
public:
    constexpr static int kWindowWidth = 1024;
    constexpr static int kWindowHeight = 768;

    Fireworks()
    {
        m_RndrContext = std::make_unique<rndr::RndrContext>();
        m_Window = m_RndrContext->CreateWin(kWindowWidth, kWindowHeight);
        assert(m_Window.IsValid());
        m_GraphicsContext = m_RndrContext->CreateGraphicsContext();
        assert(m_GraphicsContext.IsValid());
        const rndr::SwapChainProperties SwapChainProps{.UseDepthStencil = true, .IsWindowed = true};
        m_SwapChain = m_GraphicsContext->CreateSwapChain(
            m_Window->GetNativeWindowHandle(), kWindowWidth, kWindowHeight, SwapChainProps);
        assert(m_SwapChain.IsValid());
        m_Renderer = std::make_unique<physics::Renderer>(m_GraphicsContext.Get(), kWindowWidth,
                                                         kWindowHeight);
        assert(m_Renderer);

        rndr::WindowDelegates::OnResize.Add(
            [this](rndr::Window* Window, int Width, int Height)
            {
                if (m_Window.Get() != Window)
                {
                    return;
                }
                m_SwapChain->FrameBuffer->Resize(m_GraphicsContext.Get(), Width, Height);
                m_Renderer->Resize(Width, Height);
            });
    }

    void Run()
    {
        while (!m_Window->IsClosed())
        {
            m_Window->ProcessEvents();

            m_GraphicsContext->ClearColor(m_SwapChain->FrameBuffer->ColorBuffers[0].Get(),
                                          rndr::Colors::kGreen);
            m_GraphicsContext->ClearDepth(m_SwapChain->FrameBuffer->DepthStencilBuffer.Get(), 1.0f);

            m_GraphicsContext->Present(m_SwapChain.Get(), false);
        }
    }

private:
    std::unique_ptr<rndr::RndrContext> m_RndrContext;
    rndr::ScopePtr<rndr::Window> m_Window;
    rndr::ScopePtr<rndr::SwapChain> m_SwapChain;
    rndr::ScopePtr<rndr::GraphicsContext> m_GraphicsContext;
    std::unique_ptr<physics::Renderer> m_Renderer;
};

int main()
{
    Fireworks App;
    App.Run();
    return 0;
}
