#include "physics/particle-contacts.h"

#include "physics/particle.h"

physics::ParticleContact::ParticleContact(physics::Particle* main_particle,
                                          physics::Particle* other_particle,
                                          physics::real restitution,
                                          physics::real penetration)
    : m_main_particle(main_particle),
      m_other_particle(other_particle),
      m_restitution(restitution),
      m_penetration(penetration)
{
    if (m_main_particle == nullptr || m_other_particle == nullptr)
    {
        m_contact_normal = math::Vector3::Zero;
    }
    else
    {
        m_contact_normal = main_particle->GetPosition() - other_particle->GetPosition();
        constexpr real k_epsilon = PHYSICS_REALC(1e-6);
        if (!math::IsEqual(m_contact_normal, math::Vector3::Zero, k_epsilon))
        {
            m_contact_normal = math::Normalize(m_contact_normal);
        }
    }
}

physics::ParticleContact::ParticleContact(physics::Particle* main_particle,
                                          physics::real restitution,
                                          physics::real penetration,
                                          const math::Vector3& contact_normal)
    : m_main_particle(main_particle),
      m_restitution(restitution),
      m_penetration(penetration),
      m_contact_normal(contact_normal)
{
    constexpr real k_epsilon = PHYSICS_REALC(1e-6);
    if (!math::IsEqual(m_contact_normal, math::Vector3::Zero, k_epsilon))
    {
        m_contact_normal = math::Normalize(m_contact_normal);
    }
}

physics::ParticleContact::ParticleContact(physics::Particle* main_particle,
                                          physics::Particle* other_particle,
                                          physics::real restitution,
                                          physics::real penetration,
                                          const math::Vector3& contact_normal)
    : m_main_particle(main_particle),
      m_other_particle(other_particle),
      m_restitution(restitution),
      m_penetration(penetration),
      m_contact_normal(contact_normal)
{
    constexpr real k_epsilon = PHYSICS_REALC(1e-6);
    if (!math::IsEqual(m_contact_normal, math::Vector3::Zero, k_epsilon))
    {
        m_contact_normal = math::Normalize(m_contact_normal);
    }
}

bool physics::ParticleContact::IsValid() const
{
    if (m_main_particle == nullptr)
    {
        return false;
    }
    constexpr real k_epsilon = PHYSICS_REALC(1e-6);
    if (math::IsEqual(m_contact_normal, math::Vector3::Zero, k_epsilon))
    {
        return false;
    }
    if (m_restitution < PHYSICS_REALC(0.0) || m_restitution > PHYSICS_REALC(1.0))
    {
        return false;
    }
    if (math::IsNaN(m_penetration) || math::IsNaN(m_restitution) || m_contact_normal.HasNaNs())
    {
        return false;
    }
    return true;
}

void physics::ParticleContact::Resolve(physics::real delta_seconds)
{
    ResolveVelocity(delta_seconds);
    ResolveInterpenetration(delta_seconds);
}

void physics::ParticleContact::ResolveVelocity(physics::real delta_seconds)
{
    PHYSICS_UNUSED(delta_seconds);

    const real separating_velocity = CalculateSeparatingVelocity();

    // If the particles are moving apart, do nothing.
    if (separating_velocity > PHYSICS_REALC(0.0))
    {
        return;
    }

    real new_separating_velocity = -separating_velocity * m_restitution;

    math::Vector3 last_frame_acceleration = m_main_particle->GetAcceleration();
    if (m_other_particle != nullptr)
    {
        last_frame_acceleration -= m_other_particle->GetAcceleration();
    }
    const real last_frame_separating_velocity =
        math::Dot(last_frame_acceleration * delta_seconds, m_contact_normal);

    // Remove negative separating velocity due to acceleration of the last frame. This helps to
    // remove rest contact jitter.
    if (last_frame_separating_velocity < PHYSICS_REALC(0.0))
    {
        new_separating_velocity += m_restitution * last_frame_separating_velocity;
        if (new_separating_velocity < PHYSICS_REALC(0.0))
        {
            new_separating_velocity = PHYSICS_REALC(0.0);
        }
    }

    const real delta_velocity = new_separating_velocity - separating_velocity;

    real total_inverse_mass = m_main_particle->GetInverseMass();
    if (m_other_particle != nullptr)
    {
        total_inverse_mass += m_other_particle->GetInverseMass();
    }

    // Both particles have infinite mass, so do nothing.
    if (total_inverse_mass <= PHYSICS_REALC(0.0))
    {
        return;
    }

    // Calculate the impulse to apply.
    const real total_impulse = delta_velocity / total_inverse_mass;

    // Find the amount of impulse per unit of inverse mass.
    const math::Vector3 total_impulse_vector = m_contact_normal * total_impulse;

    // Apply impulses: they are applied in the direction of the contact, and are proportional to the
    // inverse mass.
    m_main_particle->SetVelocity(m_main_particle->GetVelocity() +
                                 total_impulse_vector * m_main_particle->GetInverseMass());
    if (m_other_particle != nullptr)
    {
        // We subtract here since we are applying the impulse in the opposite direction of the
        // contact normal.
        m_other_particle->SetVelocity(m_other_particle->GetVelocity() -
                                      total_impulse_vector * m_other_particle->GetInverseMass());
    }
}

void physics::ParticleContact::ResolveInterpenetration(physics::real delta_seconds)
{
    PHYSICS_UNUSED(delta_seconds);

    // There is no interpenetration, so do nothing.
    if (m_penetration <= PHYSICS_REALC(0.0))
    {
        return;
    }

    real total_inverse_mass = m_main_particle->GetInverseMass();
    if (m_other_particle != nullptr)
    {
        total_inverse_mass += m_other_particle->GetInverseMass();
    }

    // Both particles are immovable objects, so do nothing.
    if (total_inverse_mass <= PHYSICS_REALC(0.0))
    {
        return;
    }

    const math::Vector3 move_per_inverse_mass =
        m_contact_normal * (m_penetration / total_inverse_mass);

    const math::Vector3 main_delta_movement =
        move_per_inverse_mass * m_main_particle->GetInverseMass();
    math::Vector3 other_delta_movement = math::Vector3::Zero;
    if (m_other_particle != nullptr)
    {
        other_delta_movement = -move_per_inverse_mass * m_other_particle->GetInverseMass();
    }

    m_main_particle->SetPosition(m_main_particle->GetPosition() + main_delta_movement);
    if (m_other_particle != nullptr)
    {
        m_other_particle->SetPosition(m_other_particle->GetPosition() + other_delta_movement);
    }
}

physics::real physics::ParticleContact::CalculateSeparatingVelocity() const
{
    math::Vector3 relative_velocity = m_main_particle->GetVelocity();
    if (m_other_particle != nullptr)
    {
        relative_velocity -= m_other_particle->GetVelocity();
    }

    return math::Dot(relative_velocity, m_contact_normal);
}

void physics::ParticleContactResolver::ResolveContacts(Span<ParticleContact> contacts,
                                                       physics::real delta_seconds)
{
    if (contacts.empty())
    {
        return;
    }

    m_iterations_used = 0;
    while (m_iterations_used < m_iterations)
    {
        real smallest_separating_velocity = math::kInfinity;
        ParticleContact* smallest_contact = nullptr;
        for (ParticleContact& contact : contacts)
        {
            const real separating_velocity = contact.CalculateSeparatingVelocity();
            const bool is_actual_contact = separating_velocity < PHYSICS_REALC(0.0) ||
                                         contact.GetPenetration() > PHYSICS_REALC(0.0);
            if (separating_velocity < smallest_separating_velocity && is_actual_contact)
            {
                smallest_separating_velocity = separating_velocity;
                smallest_contact = &contact;
            }
        }
        // No contacts left to resolve.
        if (smallest_contact == nullptr)
        {
            break;
        }

        smallest_contact->Resolve(delta_seconds);

        m_iterations_used++;
    }
}
