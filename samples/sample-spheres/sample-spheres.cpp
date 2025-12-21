#include "opal/time.h"

#include "rndr/application.hpp"
#include "rndr/graphics-types.hpp"
#include "rndr/render-api.hpp"
#include "rndr/fly-camera.hpp"

#include "physics/core.hpp"

#include "shared/player-controller.h"

class SampleApp
{
public:
    SampleApp()
    {
        m_rndr_app = Rndr::Application::Create({.enable_input_system = true});
        m_window = m_rndr_app->CreateGenericWindow({.width = 1920, .height = 1080, .name = "Spheres Sample"});
        m_graphics_context = Rndr::GraphicsContext({.enable_debug_layer = true, .window_handle = m_window->GetNativeHandle()});
        m_swap_chain = Rndr::SwapChain(m_graphics_context, {.width = 1920, .height = 1080, .enable_vsync = true});

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
                                    Rndr::InputPrimitive::P, Rndr::InputTrigger::ButtonReleased,
                                    [this](Rndr::InputPrimitive, Rndr::InputTrigger, Rndr::f32, bool) { ResetSimulation(); })});

        const Rndr::FlyCameraDesc fly_camera_desc{.start_position = {0.0f, 1.0f, 0.0f}, .start_yaw_radians = 0};
        PlayerController controller(m_rndr_app, 1920, 1080, fly_camera_desc, 10.0f, 0.005f, 0.005f);
    }

    ~SampleApp()
    {
        m_graphics_context.Destroy();
        m_rndr_app->DestroyGenericWindow(m_window.GetPtr());
        m_rndr_app->Destroy();
    }

    void Run()
    {
        Rndr::f32 delta_seconds = 0.016f;
        while (!m_window->IsClosed())
        {
            const Rndr::f64 start_seconds = Opal::GetSeconds();
            m_rndr_app->ProcessSystemEvents(delta_seconds);
            SimulateFrame(delta_seconds);
            RenderFrame(delta_seconds);
            const Rndr::f64 end_seconds = Opal::GetSeconds();
            delta_seconds = static_cast<Rndr::f32>(end_seconds - start_seconds);
        }
    }

    virtual void SimulateFrame(Physics::f32 delta_seconds) = 0;

    void RenderFrame(Physics::f32 delta_seconds)
    {

    }

    void ToggleMovementControls() {}
    void PauseSimulation() {}
    void ResetSimulation() {}

private:
    Opal::Ref<Rndr::Application> m_rndr_app;
    Opal::Ref<Rndr::GenericWindow> m_window;
    Rndr::GraphicsContext m_graphics_context;
    Rndr::SwapChain m_swap_chain;
};

int main() {}