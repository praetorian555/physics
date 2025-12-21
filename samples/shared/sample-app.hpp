#pragma once

#include "physics/core.hpp"

#include "opal/container/ref.h"

#include "rndr/render-api.hpp"
#include "rndr/application.hpp"
#include "rndr/renderers/grid-renderer.hpp"
#include "rndr/renderers/shape-3d-renderer.hpp"

#include "shared/player-controller.hpp"

class SampleApp
{
public:
    SampleApp();
    virtual ~SampleApp();

    virtual void SetupSimulation() {}
    void Run();
    void ConditionallySimulateFrame(Physics::f32 delta_seconds);
    virtual void SimulateFrame(Physics::f32) {}
    void RenderFrame(Physics::f32 delta_seconds);
    virtual void RenderImGui(Physics::f32, Rndr::CommandList&) {}
    void ToggleMovementControls();
    void PauseSimulation();
    void AdvanceSimulationFrame();
    virtual void ResetSimulation() {}

private:
    Opal::Ref<Rndr::Application> m_rndr_app;
    Opal::Ref<Rndr::GenericWindow> m_window;
    Opal::Ref<Rndr::GraphicsContext> m_graphics_context;
    Opal::Ref<Rndr::SwapChain> m_swap_chain;
    Opal::Ref<PlayerController> m_player_controller;
    Opal::Ref<Rndr::GridRenderer> m_grid_renderer;
    Opal::Ref<Rndr::Shape3DRenderer> m_shape_renderer;

    bool m_is_paused = false;
    bool m_advance_frame = false;
};