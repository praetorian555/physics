#include "shared/sample-app.hpp"

#include "opal/paths.h"
#include "opal/time.h"

#include "physics/shapes/sphere-shape.hpp"

SampleApp::SampleApp()
{
    m_rndr_app = Rndr::Application::Create({.enable_input_system = true});
    m_window = m_rndr_app->CreateGenericWindow({.width = 1920, .height = 1080, .name = "Spheres Sample"});
    m_rndr_app->EnableHighPrecisionCursorMode(true, m_window.Get());
    m_graphics_context = Opal::New<Rndr::GraphicsContext>(
        Opal::GetDefaultAllocator(), Rndr::GraphicsContextDesc{.enable_debug_layer = true, .window_handle = m_window->GetNativeHandle()});
    m_swap_chain = Opal::New<Rndr::SwapChain>(Opal::GetDefaultAllocator(), m_graphics_context,
                                              Rndr::SwapChainDesc{.width = 1920,
                                                                  .height = 1080,
                                                                  .depth_stencil_format = Rndr::PixelFormat::D32_FLOAT_S8_UINT,
                                                                  .use_depth_stencil = true,
                                                                  .enable_vsync = false});

    // Setup input handling
    Rndr::InputSystem& input_system = m_rndr_app->GetInputSystemChecked();
    Rndr::InputContext& input_context = input_system.GetCurrentContext();
    input_context.AddAction("Toggle movement controls",
                            {Rndr::InputBinding::CreateKeyboardButtonBinding(
                                Rndr::InputPrimitive::F1, Rndr::InputTrigger::ButtonReleased,
                                [this](Rndr::InputPrimitive, Rndr::InputTrigger, Rndr::f32, bool) { ToggleMovementControls(); })});
    input_context.AddAction("Pause Simulation",
                            {Rndr::InputBinding::CreateKeyboardButtonBinding(
                                Rndr::InputPrimitive::P, Rndr::InputTrigger::ButtonReleased,
                                [this](Rndr::InputPrimitive, Rndr::InputTrigger, Rndr::f32, bool) { PauseSimulation(); })});
    input_context.AddAction("Reset Simulation",
                            {Rndr::InputBinding::CreateKeyboardButtonBinding(
                                Rndr::InputPrimitive::R, Rndr::InputTrigger::ButtonReleased,
                                [this](Rndr::InputPrimitive, Rndr::InputTrigger, Rndr::f32, bool) { ResetSimulation(); })});
    input_context.AddAction("Advance Simulation",
                            {Rndr::InputBinding::CreateKeyboardButtonBinding(
                                Rndr::InputPrimitive::N, Rndr::InputTrigger::ButtonReleased,
                                [this](Rndr::InputPrimitive, Rndr::InputTrigger, Rndr::f32, bool) { AdvanceSimulationFrame(); })});

    const Rndr::FlyCameraDesc fly_camera_desc{
        .start_position = {0.0f, 10.0f, 0.0f}, .start_yaw_radians = 0, .projection_desc = {.far = 1000.0f}};
    m_player_controller =
        New<PlayerController>(Opal::GetDefaultAllocator(), m_rndr_app, 1920, 1080, fly_camera_desc, 10.0f, 0.005f, 0.005f);

    const Rndr::RendererBaseDesc renderer_desc{.graphics_context = Opal::Ref{&m_graphics_context}, .swap_chain = Opal::Ref{&m_swap_chain}};
    const Opal::Ref<Rndr::FrameBuffer> default_framebuffer{nullptr};
    m_grid_renderer = Opal::New<Rndr::GridRenderer>(Opal::GetDefaultAllocator(), "Grid Renderer", renderer_desc, default_framebuffer);
    m_shape_renderer = Opal::New<Rndr::Shape3DRenderer>(Opal::GetDefaultAllocator(), "Shape Renderer", renderer_desc, default_framebuffer);
    Rndr::MaterialDesc material_desc{.albedo_texture_path = Opal::Paths::Combine(RNDR_CORE_ASSETS_DIR, "default-texture.png")};
    m_default_material = Opal::New<Rndr::Material>(Opal::GetDefaultAllocator(), m_graphics_context, material_desc);
}

SampleApp::~SampleApp()
{
    Opal::Delete<Rndr::Shape3DRenderer>(Opal::GetDefaultAllocator(), m_shape_renderer.GetPtr());
    Opal::Delete<Rndr::GridRenderer>(Opal::GetDefaultAllocator(), m_grid_renderer.GetPtr());
    Opal::Delete<PlayerController>(Opal::GetDefaultAllocator(), m_player_controller.GetPtr());
    Opal::Delete<Rndr::GraphicsContext>(Opal::GetDefaultAllocator(), m_graphics_context.GetPtr());
    Opal::Delete<Rndr::SwapChain>(Opal::GetDefaultAllocator(), m_swap_chain.GetPtr());
    m_rndr_app->DestroyGenericWindow(m_window.GetPtr());
    m_rndr_app->Destroy();
}

void SampleApp::Run()
{
    SetupSimulation();

    Rndr::f32 delta_seconds = 0.016f;
    while (!m_window->IsClosed())
    {
        const Rndr::f64 start_seconds = Opal::GetSeconds();
        Opal::GetScratchAllocator()->Reset();
        m_rndr_app->ProcessSystemEvents(delta_seconds);
        m_player_controller->Tick(delta_seconds);
        ConditionallySimulateFrame(delta_seconds);
        RenderFrame(delta_seconds);
        const Rndr::f64 end_seconds = Opal::GetSeconds();
        delta_seconds = static_cast<Rndr::f32>(end_seconds - start_seconds);
    }
}

void SampleApp::ConditionallySimulateFrame(Physics::f32 delta_seconds)
{
    bool should_update = false;
    m_time_until_physics_update -= delta_seconds;
    if (m_time_until_physics_update <= 0.0f)
    {
        m_time_until_physics_update = m_physics_update_interval;
        should_update = true;
    }
    if (!m_is_paused)
    {
        if (should_update)
        {
            SimulateFrame(m_physics_update_interval);
        }
    }
    else
    {
        if (m_advance_frame && should_update)
        {
            SimulateFrame(m_physics_update_interval);
            m_advance_frame = false;
        }
    }
}

void SampleApp::RenderFrame(Physics::f32 delta_seconds)
{
    Rndr::CommandList cmd_list{m_graphics_context};
    cmd_list.CmdBindSwapChainFrameBuffer(m_swap_chain);
    cmd_list.CmdClearAll(Rndr::Colors::k_black);
    m_grid_renderer->SetTransforms(m_player_controller->GetViewTransform(), m_player_controller->GetProjectionTransform());

    m_grid_renderer->Render(delta_seconds, cmd_list);
    for (const auto& body : m_scene.GetBodies())
    {
        DrawBody(body);
    }
    m_shape_renderer->SetTransforms(m_player_controller->GetViewTransform(), m_player_controller->GetProjectionTransform());
    m_shape_renderer->SetCameraPosition(m_player_controller->GetCameraPosition());
    m_shape_renderer->AddDirectionalLight(Rndr::Vector3f(1, 1, 1), Rndr::Colors::k_white);
    m_shape_renderer->AddDirectionalLight(Rndr::Vector3f(-1, -1, -1), Rndr::Colors::k_pink);
    m_shape_renderer->Render(delta_seconds, cmd_list);
    RenderImGui(delta_seconds, cmd_list);
    cmd_list.CmdPresent(m_swap_chain);
    m_graphics_context->SubmitCommandList(cmd_list);
}

void SampleApp::ToggleMovementControls()
{
    const Rndr::CursorPositionMode mode = m_rndr_app->GetCursorPositionMode();
    if (mode == Rndr::CursorPositionMode::Normal)
    {
        m_rndr_app->ShowCursor(false);
        m_rndr_app->SetCursorPositionMode(Rndr::CursorPositionMode::ResetToCenter);
    }
    else
    {
        m_rndr_app->ShowCursor(true);
        m_rndr_app->SetCursorPositionMode(Rndr::CursorPositionMode::Normal);
    }
    m_player_controller->Enable(!m_player_controller->IsEnabled());
}

void SampleApp::PauseSimulation()
{
    m_is_paused = !m_is_paused;
}

void SampleApp::AdvanceSimulationFrame()
{
    m_advance_frame = true;
}

void SampleApp::ResetSimulation()
{
    m_scene.Reset();
    m_time_until_physics_update = m_physics_update_interval;
}

void SampleApp::AddBody(Physics::Body body)
{
    m_scene.AddBody(body);
}

void SampleApp::DrawBody(const Physics::Body& body)
{
    Physics::Matrix4x4r mat = Opal::Translate(body.position) * Opal::Rotate(body.orientation);
    switch (body.shape->GetType())
    {
        case Physics::ShapeType::Sphere:
        {
            const Physics::SphereShape* sphere = static_cast<Physics::SphereShape*>(body.shape);
            mat *= Opal::Scale(sphere->GetRadius());
            if (sphere->GetRadius() > 10.0f)
            {
                m_shape_renderer->DrawSphere(mat, m_default_material, 40, 40, 128, 128);
            }
            else
            {
                m_shape_renderer->DrawSphere(mat, m_default_material);
            }
            break;
        }
        default:
        {
            throw Opal::Exception("Invalid shape type");
        }
    }
}
