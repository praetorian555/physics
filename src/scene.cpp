#include "physics/scene.hpp"

#include <algorithm>

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
        const real mass = body.inverse_mass > 0 ? PHYSICS_CONST(1.0) / body.inverse_mass : 0;
        const Vector3r gravity_impulse = mass * m_gravity * delta_seconds;
        body.ApplyImpulseLinear(gravity_impulse);
    }

    Opal::DynamicArray<Contact> contacts;
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
                contacts.PushBack(std::move(contact));
            }
        }
    }

    if (contacts.GetSize() > 1)
    {
        std::ranges::sort(contacts, [](const Contact& a, const Contact& b)
        {
            return a.time_of_impact <= b.time_of_impact;
        });
    }

    f32 accumulated_time = 0;
    for (Contact& contact : contacts)
    {
        const f32 delta_time = contact.time_of_impact - accumulated_time;
        const Body& a = contact.a;
        const Body& b = contact.b;
        if (0 == a.inverse_mass && 0 == b.inverse_mass)
        {
            continue;
        }
        for (Body& body : m_bodies)
        {
            body.Update(delta_time);
        }

        ResolveContact(contact);
        accumulated_time += delta_time;
#ifdef OPAL_DEBUG
        m_last_simulation_frame_contacts.PushBack(contact);
#endif
    }

    const f32 remaining_time = delta_seconds - accumulated_time;
    if (remaining_time > 0.0f)
    {
        for (Body& body : m_bodies)
        {
            body.Update(remaining_time);
        }
    }
}