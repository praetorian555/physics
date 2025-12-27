#include "physics/scene.hpp"

#include "physics/intersect.hpp"

void Physics::Scene::Update(f32 delta_seconds)
{
#ifdef OPAL_DEBUG
    m_last_simulation_frame_contacts.Clear();
#endif

    for (Body& body : m_bodies)
    {
        // Calculate impulse of a gravity force
        // J = dp => F = dp / dt => dp = F * dt => J = F * dt => J = m * g * dt
        const real mass = PHYSICS_CONST(1.0) / body.inverse_mass;
        const Vector3r gravity_impulse = mass * m_gravity * delta_seconds;
        body.ApplyImpulseLinear(gravity_impulse);
    }

    for (i32 i = 0; i < m_bodies.GetSize(); i++)
    {
        for (i32 j = i + 1; j < m_bodies.GetSize(); j++)
        {
            Body& a = m_bodies[i];
            Body& b = m_bodies[j];
            if (0 == a.inverse_mass && 0 == b.inverse_mass)
            {
                continue;
            }
            Contact contact;
            if (Intersect(a, b, delta_seconds, contact))
            {
                ResolveContact(contact);
#ifdef OPAL_DEBUG
                m_last_simulation_frame_contacts.PushBack(contact);
#endif
            }
        }
    }

    for (Body& body : m_bodies)
    {
        body.Update(delta_seconds);
    }
}