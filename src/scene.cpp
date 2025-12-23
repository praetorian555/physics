#include "physics/scene.hpp"

void Physics::Scene::Update(f32 delta_seconds)
{
    for (Body& body : m_bodies)
    {
        // Calculate impulse of a gravity force
        // J = dp => F = dp / dt => dp = F * dt => J = F * dt => J = m * g * dt
        const real mass = PHYSICS_CONST(1.0) / body.inverse_mass;
        const Vector3r gravity_impulse = mass * m_gravity * delta_seconds;
        body.ApplyImpulseLinear(gravity_impulse);
    }

    for (Body& body : m_bodies)
    {
        body.position += body.linear_velocity * delta_seconds;
    }
}