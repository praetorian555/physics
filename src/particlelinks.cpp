#include "physics/particlelinks.h"

#include "physics/particle.h"

physics::ParticleLink::ParticleLink(physics::Particle* FirstParticle,
                                    physics::Particle* SecondParticle)
    : m_FirstParticle(FirstParticle), m_SecondParticle(SecondParticle)
{
}

physics::real physics::ParticleLink::GetCurrentLength() const
{
    return math::Distance(m_FirstParticle->GetPosition(), m_SecondParticle->GetPosition());
}

physics::ParticleCable::ParticleCable(physics::Particle* FirstParticle,
                                      physics::Particle* SecondParticle,
                                      physics::real MaxLength,
                                      physics::real Restitution)
    : ParticleLink(FirstParticle, SecondParticle),
      m_MaxLength(MaxLength),
      m_Restitution(Restitution)
{
}
uint32_t physics::ParticleCable::AddContact(Span<ParticleContact> Contacts)
{
    if (Contacts.empty())
    {
        return 0;
    }

    const real CurrentLength = GetCurrentLength();
    if (CurrentLength < m_MaxLength)
    {
        return 0;
    }

    ParticleContact* Contact = Contacts.data();
    *Contact = ParticleContact(m_FirstParticle, m_SecondParticle, m_Restitution,
                               CurrentLength - m_MaxLength);
    return 1;
}

physics::ParticleRod::ParticleRod(physics::Particle* FirstParticle,
                                  physics::Particle* SecondParticle,
                                  physics::real Length)
    : ParticleLink(FirstParticle, SecondParticle), m_Length(Length)
{
}

uint32_t physics::ParticleRod::AddContact(physics::Span<physics::ParticleContact> Contacts)
{
    if (Contacts.empty())
    {
        return 0;
    }

    const real CurrentLength = GetCurrentLength();
    if (CurrentLength == m_Length)
    {
        return 0;
    }

    const math::Vector3 Normal =
        math::Normalize(m_SecondParticle->GetPosition() - m_FirstParticle->GetPosition());
    ParticleContact* Contact = Contacts.data();
    if (CurrentLength > m_Length)
    {
        *Contact = ParticleContact(m_FirstParticle, m_SecondParticle, PHYSICS_REALC(0.0),
                                   CurrentLength - m_Length, Normal);
    }
    else
    {
        *Contact = ParticleContact(m_FirstParticle, m_SecondParticle, PHYSICS_REALC(0.0),
                                   m_Length - CurrentLength, -Normal);
    }

    return 1;
}
