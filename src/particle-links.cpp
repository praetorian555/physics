#include "physics/particle-links.h"

#include "physics/particle.h"

Physics::ParticleLink::ParticleLink(Physics::Particle* first_particle,
                                    Physics::Particle* second_particle)
    : m_first_particle(first_particle), m_second_particle(second_particle)
{
}

Physics::real Physics::ParticleLink::GetCurrentLength() const
{
    return math::Distance(m_first_particle->GetPosition(), m_second_particle->GetPosition());
}

Physics::ParticleCable::ParticleCable(Physics::Particle* first_particle,
                                      Physics::Particle* second_particle,
                                      Physics::real max_length,
                                      Physics::real restitution)
    : ParticleLink(first_particle, second_particle),
      m_max_length(max_length),
      m_restitution(restitution)
{
}
uint32_t Physics::ParticleCable::AddContact(Span<ParticleContact> contacts)
{
    if (contacts.empty())
    {
        return 0;
    }

    const real current_length = GetCurrentLength();
    if (current_length < m_max_length)
    {
        return 0;
    }

    ParticleContact* contact = contacts.data();
    *contact = ParticleContact(m_first_particle, m_second_particle, m_restitution,
                               current_length - m_max_length);
    return 1;
}

Physics::ParticleRod::ParticleRod(Physics::Particle* first_particle,
                                  Physics::Particle* second_particle,
                                  Physics::real length)
    : ParticleLink(first_particle, second_particle), m_length(length)
{
}

uint32_t Physics::ParticleRod::AddContact(Physics::Span<Physics::ParticleContact> contacts)
{
    if (contacts.empty())
    {
        return 0;
    }

    const real current_length = GetCurrentLength();
    if (current_length == m_length)
    {
        return 0;
    }

    const math::Vector3 normal =
        math::Normalize(m_second_particle->GetPosition() - m_first_particle->GetPosition());
    ParticleContact* contact = contacts.data();
    if (current_length > m_length)
    {
        *contact = ParticleContact(m_first_particle, m_second_particle, PHYSICS_REALC(0.0),
                                   current_length - m_length, normal);
    }
    else
    {
        *contact = ParticleContact(m_first_particle, m_second_particle, PHYSICS_REALC(0.0),
                                   m_length - current_length, -normal);
    }

    return 1;
}
