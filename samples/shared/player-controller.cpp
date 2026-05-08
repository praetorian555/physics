#include "shared/player-controller.hpp"

PlayerController::PlayerController(Rndr::Application& app, Rndr::i32 screen_width, Rndr::i32 screen_height,
                                     const Rndr::FlyCameraDesc& camera_desc, Rndr::f32 move_speed, Rndr::f32 yaw_speed,
                                     Rndr::f32 pitch_speed)
    : m_fly_camera(screen_width, screen_height, camera_desc),
      m_input_context("Player Controller Input Context"),
      m_move_speed(move_speed),
      m_yaw_speed(yaw_speed),
      m_pitch_speed(pitch_speed)
{
    m_input_context.SetEnabled(false);

    using K = Rndr::Key;
    using T = Rndr::Trigger;

    auto make_button_handler = [](Rndr::f32* target, Rndr::f32 value_on_press)
    {
        return [target, value_on_press](T trigger, bool)
        {
            *target = (trigger == T::Pressed) ? value_on_press : 0.0f;
        };
    };

    m_input_context.AddAction("MoveForwardW")
        .OnButton(make_button_handler(&m_forward_value, m_move_speed))
        .Bind(K::W, T::Pressed)
        .Bind(K::W, T::Released);
    m_input_context.AddAction("MoveForwardS")
        .OnButton(make_button_handler(&m_forward_value, -m_move_speed))
        .Bind(K::S, T::Pressed)
        .Bind(K::S, T::Released);
    m_input_context.AddAction("MoveRightD")
        .OnButton(make_button_handler(&m_right_value, m_move_speed))
        .Bind(K::D, T::Pressed)
        .Bind(K::D, T::Released);
    m_input_context.AddAction("MoveRightA")
        .OnButton(make_button_handler(&m_right_value, -m_move_speed))
        .Bind(K::A, T::Pressed)
        .Bind(K::A, T::Released);

    m_input_context.AddAction("LookUp")
        .OnButton(make_button_handler(&m_vertical_value, m_pitch_speed))
        .Bind(K::UpArrow, T::Pressed)
        .Bind(K::UpArrow, T::Released);
    m_input_context.AddAction("LookDown")
        .OnButton(make_button_handler(&m_vertical_value, -m_pitch_speed))
        .Bind(K::DownArrow, T::Pressed)
        .Bind(K::DownArrow, T::Released);
    m_input_context.AddAction("LookLeft")
        .OnButton(make_button_handler(&m_horizontal_value, m_yaw_speed))
        .Bind(K::LeftArrow, T::Pressed)
        .Bind(K::LeftArrow, T::Released);
    m_input_context.AddAction("LookRight")
        .OnButton(make_button_handler(&m_horizontal_value, -m_yaw_speed))
        .Bind(K::RightArrow, T::Pressed)
        .Bind(K::RightArrow, T::Released);

    m_input_context.AddAction("MouseLookVert")
        .OnMousePosition(
            [this](Rndr::MouseAxis axis, Rndr::f32 delta)
            {
                if (axis == Rndr::MouseAxis::Y)
                {
                    m_fly_camera.AddPitch(-m_pitch_speed * delta);
                }
            })
        .Bind(Rndr::MouseAxis::Y);
    m_input_context.AddAction("MouseLookHorz")
        .OnMousePosition(
            [this](Rndr::MouseAxis axis, Rndr::f32 delta)
            {
                if (axis == Rndr::MouseAxis::X)
                {
                    m_fly_camera.AddYaw(-m_yaw_speed * delta);
                }
            })
        .Bind(Rndr::MouseAxis::X);

    app.GetInputSystemChecked().PushContext(m_input_context);
}

void PlayerController::Enable(bool enable)
{
    m_input_context.SetEnabled(enable);
}

bool PlayerController::IsEnabled() const
{
    return m_input_context.IsEnabled();
}

void PlayerController::SetScreenSize(Rndr::i32 width, Rndr::i32 height)
{
    m_fly_camera.SetScreenSize(width, height);
}

void PlayerController::Tick(Rndr::f32 delta_seconds)
{
    if (m_vertical_value != 0)
    {
        m_fly_camera.AddPitch(m_vertical_value);
    }
    if (m_horizontal_value != 0)
    {
        m_fly_camera.AddYaw(m_horizontal_value);
    }
    if (m_forward_value != 0)
    {
        m_fly_camera.MoveForward(m_forward_value);
    }
    if (m_right_value != 0)
    {
        m_fly_camera.MoveRight(m_right_value);
    }

    m_fly_camera.Tick(delta_seconds);
}