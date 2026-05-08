#include "shared/sample-app.hpp"

#include "opal/time.h"

#include "physics/shapes/box-shape.hpp"
#include "physics/shapes/convex-shape.hpp"
#include "physics/shapes/sphere-shape.hpp"

#include "rndr/canvas/vertex-layout.hpp"
#include "rndr/colors.hpp"
#include "rndr/generic-window.hpp"

namespace
{
constexpr Rndr::i32 k_screen_width = 1920;
constexpr Rndr::i32 k_screen_height = 1080;
}  // namespace

SampleApp::SampleApp()
    : m_rndr_app(Rndr::Application::Create({.enable_input_system = true})),
      m_window(m_rndr_app->CreateGenericWindow({.width = k_screen_width, .height = k_screen_height, .name = "Physics Sample"})),
      m_context(Rndr::Canvas::Context::Init(m_window.Clone(), {.vsync_enabled = false})),
      m_player_controller(Opal::MakeScoped<PlayerController>(
          Opal::GetDefaultAllocator(), *m_rndr_app, k_screen_width, k_screen_height,
          Rndr::FlyCameraDesc{.start_position = {0.0f, 10.0f, 0.0f}, .start_yaw_radians = 0, .projection_desc = {.far = 1000.0f}}, 10.0f,
          0.005f, 0.005f)),
      m_grid_renderer(Opal::Ref<Rndr::Canvas::Context>{&m_context}),
      m_pbr_renderer(Opal::Ref<Rndr::Canvas::Context>{&m_context})
{
    m_window->EnableHighPrecisionCursorMode(true);

    m_default_material.albedo_color = Rndr::Vector4f{0.7f, 0.7f, 0.75f, 1.0f};
    m_default_material.roughness = Rndr::Vector4f{0.5f, 0.5f, 0.0f, 0.0f};
    m_default_material.metallic_factor = 0.1f;

    m_scratch_allocator = Opal::MakeScoped<Opal::LinearAllocator>(nullptr, "Scratch Allocator");
    Opal::PushScratchAllocator(m_scratch_allocator.Get());

    Rndr::InputContext& input_context = m_rndr_app->GetInputSystemChecked().GetContextByName("Default");
    auto on_release = [](auto fn)
    {
        return [fn](Rndr::Trigger trigger, bool)
        {
            if (trigger == Rndr::Trigger::Released)
                fn();
        };
    };
    input_context.AddAction("Toggle movement controls")
        .OnButton(on_release([this] { ToggleMovementControls(); }))
        .Bind(Rndr::Key::F1, Rndr::Trigger::Released);
    input_context.AddAction("Pause Simulation")
        .OnButton(on_release([this] { PauseSimulation(); }))
        .Bind(Rndr::Key::P, Rndr::Trigger::Released);
    input_context.AddAction("Reset Simulation")
        .OnButton(on_release([this] { ResetSimulation(); }))
        .Bind(Rndr::Key::R, Rndr::Trigger::Released);
    input_context.AddAction("Advance Simulation")
        .OnButton(on_release([this] { AdvanceSimulationFrame(); }))
        .Bind(Rndr::Key::N, Rndr::Trigger::Released);
    input_context.AddAction("Exit")
        .OnButton(on_release([this] { m_window->RequestClose(); }))
        .Bind(Rndr::Key::Escape, Rndr::Trigger::Released);

    m_rndr_app->on_window_resize.Bind(
        [this](const Rndr::GenericWindow& w, Rndr::i32 width, Rndr::i32 height)
        {
            if (m_window.GetPtr() == &w)
            {
                m_context.Resize(width, height);
                m_player_controller->SetScreenSize(width, height);
            }
        });
}

SampleApp::~SampleApp() = default;

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

void SampleApp::RenderFrame(Physics::f32)
{
    const Rndr::Matrix4x4f view = m_player_controller->GetViewTransform();
    const Rndr::Matrix4x4f projection = m_player_controller->GetProjectionTransform();

    Rndr::Canvas::DrawList draw_list;
    draw_list.SetRenderTarget(m_context);
    draw_list.Clear({0.0f, 0.0f, 0.0f, 1.0f}, 1.0f);

    m_grid_renderer.Render(draw_list, view, projection);

    m_pbr_renderer.BeginFrame();
    m_pbr_renderer.SetViewProjection(projection * view);
    m_pbr_renderer.SetCameraPosition(m_player_controller->GetCameraPosition());
    m_pbr_renderer.AddDirectionalLight(Rndr::Vector3f(1, 1, 1), Rndr::Colors::k_white);
    m_pbr_renderer.AddDirectionalLight(Rndr::Vector3f(-1, -1, -1), Rndr::Colors::k_pink);
    for (const auto& body : m_scene.GetBodies())
    {
        DrawBody(body);
    }
    m_pbr_renderer.Render(draw_list);

    draw_list.Execute();
    m_context.Present();
}

void SampleApp::ToggleMovementControls()
{
    const bool enabled = !m_player_controller->IsEnabled();
    m_rndr_app->ShowCursor(!enabled);
    m_player_controller->Enable(enabled);
    const Rndr::CursorPositionMode mode = m_window->GetCursorPositionMode();
    m_window->SetCursorPositionMode(mode == Rndr::CursorPositionMode::Normal ? Rndr::CursorPositionMode::ResetToCenter
                                                                             : Rndr::CursorPositionMode::Normal);
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
            const Physics::real r = sphere->GetRadius();
            mat *= Opal::Scale(r);
            const bool large = r > 10.0f;
            m_pbr_renderer.DrawSphere(mat, m_default_material, 1.0f, 1.0f, large ? 64 : 32, large ? 64 : 32);
            break;
        }
        case Physics::ShapeType::Box:
        {
            const Physics::BoxShape* box = static_cast<Physics::BoxShape*>(body.shape);
            const Physics::Vector3r extent = box->GetExtent();
            mat *= Opal::Scale(extent.x, extent.y, extent.z);
            m_pbr_renderer.DrawCube(mat, m_default_material);
            break;
        }
        case Physics::ShapeType::Convex:
        {
            const Rndr::Canvas::Mesh& mesh = m_meshes.GetValue(body.shape);
            char key_buf[32];
            std::snprintf(key_buf, sizeof(key_buf), "convex_%p", static_cast<const void*>(body.shape));
            m_pbr_renderer.DrawMesh(Opal::StringUtf8(key_buf), mesh, mat, m_default_material);
            break;
        }
        default:
        {
            throw Opal::Exception("Invalid shape type");
        }
    }
}

void SampleApp::AddMesh(Physics::Shape* shape, Rndr::Canvas::Mesh mesh)
{
    m_meshes.Insert(shape, std::move(mesh));
}