#include "physics/particle-links.h"

#include "physics/particle.h"

physics::ParticleLink::ParticleLink(physics::Particle* first_particle,
                                    physics::Particle* second_particle)
    : m_first_particle(first_particle), m_second_particle(second_particle)
{
}

physics::real physics::ParticleLink::GetCurrentLength() const
{
    return math::Distance(m_first_particle->GetPosition(), m_second_particle->GetPosition());
}

physics::ParticleCable::ParticleCable(physics::Particle* first_particle,
                                      physics::Particle* second_particle,
                                      physics::real max_length,
                                      physics::real restitution)
    : ParticleLink(first_particle, second_particle),
      m_max_length(max_length),
      m_restitution(restitution)
{
}
uint32_t physics::ParticleCable::AddContact(Span<ParticleContact> contacts)
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

physics::ParticleRod::ParticleRod(physics::Particle* first_particle,
                                  physics::Particle* second_particle,
                                  physics::real length)
    : ParticleLink(first_particle, second_particle), m_length(length)
{
}

uint32_t physics::ParticleRod::AddContact(physics::Span<physics::ParticleContact> contacts)
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
