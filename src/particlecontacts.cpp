#include "physics/particlecontacts.h"

#include "physics/particle.h"

physics::ParticleContact::ParticleContact(physics::Particle* MainParticle,
                                          physics::Particle* OtherParticle,
                                          physics::real Restitution,
                                          physics::real Penetration)
    : m_MainParticle(MainParticle),
      m_OtherParticle(OtherParticle),
      m_Restitution(Restitution),
      m_Penetration(Penetration)
{
    if (m_MainParticle == nullptr || m_OtherParticle == nullptr)
    {
        m_ContactNormal = math::Vector3::Zero;
    }
    else
    {
        m_ContactNormal = MainParticle->GetPosition() - OtherParticle->GetPosition();
        m_ContactNormal = math::Normalize(m_ContactNormal);
    }
}

physics::ParticleContact::ParticleContact(physics::Particle* MainParticle,
                                          physics::real Restitution,
                                          physics::real Penetration,
                                          const math::Vector3& ContactNormal)
    : m_MainParticle(MainParticle),
      m_Restitution(Restitution),
      m_Penetration(Penetration),
      m_ContactNormal(ContactNormal)
{
    if (m_ContactNormal != math::Vector3::Zero)
    {
        m_ContactNormal = math::Normalize(m_ContactNormal);
    }
}

bool physics::ParticleContact::IsValid() const
{
    if (m_MainParticle == nullptr)
    {
        return false;
    }
    if (m_ContactNormal == math::Vector3::Zero)
    {
        return false;
    }
    if (m_Restitution < PHYSICS_REALC(0.0) || m_Restitution > PHYSICS_REALC(1.0))
    {
        return false;
    }
    if (math::IsNaN(m_Penetration) || math::IsNaN(m_Restitution) || m_ContactNormal.HasNaNs())
    {
        return false;
    }
    return true;
}

void physics::ParticleContact::Resolve(physics::real DeltaSeconds)
{
    ResolveVelocity(DeltaSeconds);
    ResolveInterpenetration(DeltaSeconds);
}

void physics::ParticleContact::ResolveVelocity(physics::real DeltaSeconds)
{
    PHYSICS_UNUSED(DeltaSeconds);

    const real SeparatingVelocity = CalculateSeparatingVelocity();

    // If the particles are moving apart, do nothing.
    if (SeparatingVelocity > PHYSICS_REALC(0.0))
    {
        return;
    }

    real NewSeparatingVelocity = -SeparatingVelocity * m_Restitution;

    math::Vector3 LastFrameAcceleration = m_MainParticle->GetAcceleration();
    if (m_OtherParticle != nullptr)
    {
        LastFrameAcceleration -= m_OtherParticle->GetAcceleration();
    }
    const real LastFrameSeparatingVelocity =
        math::Dot(LastFrameAcceleration * DeltaSeconds, m_ContactNormal);

    // Remove negative separating velocity due to acceleration of the last frame. This helps to
    // remove rest contact jitter.
    if (LastFrameSeparatingVelocity < PHYSICS_REALC(0.0))
    {
        NewSeparatingVelocity += m_Restitution * LastFrameSeparatingVelocity;
        if (NewSeparatingVelocity < PHYSICS_REALC(0.0))
        {
            NewSeparatingVelocity = PHYSICS_REALC(0.0);
        }
    }

    const real DeltaVelocity = NewSeparatingVelocity - SeparatingVelocity;

    real TotalInverseMass = m_MainParticle->GetInverseMass();
    if (m_OtherParticle != nullptr)
    {
        TotalInverseMass += m_OtherParticle->GetInverseMass();
    }

    // Both particles have infinite mass, so do nothing.
    if (TotalInverseMass <= PHYSICS_REALC(0.0))
    {
        return;
    }

    // Calculate the impulse to apply.
    const real TotalImpulse = DeltaVelocity / TotalInverseMass;

    // Find the amount of impulse per unit of inverse mass.
    const math::Vector3 TotalImpulseVector = m_ContactNormal * TotalImpulse;

    // Apply impulses: they are applied in the direction of the contact, and are proportional to the
    // inverse mass.
    m_MainParticle->SetVelocity(m_MainParticle->GetVelocity() +
                                TotalImpulseVector * m_MainParticle->GetInverseMass());
    if (m_OtherParticle != nullptr)
    {
        // We subtract here since we are applying the impulse in the opposite direction of the
        // contact normal.
        m_OtherParticle->SetVelocity(m_OtherParticle->GetVelocity() -
                                     TotalImpulseVector * m_OtherParticle->GetInverseMass());
    }
}

void physics::ParticleContact::ResolveInterpenetration(physics::real DeltaSeconds)
{
    PHYSICS_UNUSED(DeltaSeconds);

    // There is no interpenetration, so do nothing.
    if (m_Penetration <= PHYSICS_REALC(0.0))
    {
        return;
    }

    real TotalInverseMass = m_MainParticle->GetInverseMass();
    if (m_OtherParticle != nullptr)
    {
        TotalInverseMass += m_OtherParticle->GetInverseMass();
    }

    // Both particles are immovable objects, so do nothing.
    if (TotalInverseMass <= PHYSICS_REALC(0.0))
    {
        return;
    }

    const math::Vector3 MovePerInverseMass = m_ContactNormal * (m_Penetration / TotalInverseMass);

    const math::Vector3 MainDeltaMovement = MovePerInverseMass * m_MainParticle->GetInverseMass();
    math::Vector3 OtherDeltaMovement = math::Vector3::Zero;
    if (m_OtherParticle != nullptr)
    {
        OtherDeltaMovement = -MovePerInverseMass * m_OtherParticle->GetInverseMass();
    }

    m_MainParticle->SetPosition(m_MainParticle->GetPosition() + MainDeltaMovement);
    if (m_OtherParticle != nullptr)
    {
        m_OtherParticle->SetPosition(m_OtherParticle->GetPosition() + OtherDeltaMovement);
    }
}

physics::real physics::ParticleContact::CalculateSeparatingVelocity() const
{
    math::Vector3 RelativeVelocity = m_MainParticle->GetVelocity();
    if (m_OtherParticle != nullptr)
    {
        RelativeVelocity -= m_OtherParticle->GetVelocity();
    }

    return math::Dot(RelativeVelocity, m_ContactNormal);
}

void physics::ParticleContactResolver::ResolveContacts(Span<ParticleContact> Contacts,
                                                       physics::real DeltaSeconds)
{
    if (Contacts.empty())
    {
        return;
    }

    m_IterationsUsed = 0;
    while (m_IterationsUsed < m_Iterations)
    {
        real SmallestSeparatingVelocity = math::kInfinity;
        ParticleContact* SmallestContact = nullptr;
        for (ParticleContact& Contact : Contacts)
        {
            const real SeparatingVelocity = Contact.CalculateSeparatingVelocity();
            const bool IsActualContact = SeparatingVelocity < PHYSICS_REALC(0.0) ||
                                         Contact.GetPenetration() > PHYSICS_REALC(0.0);
            if (SeparatingVelocity < SmallestSeparatingVelocity && IsActualContact)
            {
                SmallestSeparatingVelocity = SeparatingVelocity;
                SmallestContact = &Contact;
            }
        }
        // No contacts left to resolve.
        if (SmallestContact == nullptr)
        {
            break;
        }

        SmallestContact->Resolve(DeltaSeconds);

        m_IterationsUsed++;
    }
}

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
uint32_t physics::ParticleCable::AddContact(Span<ParticleContact> Contacts, uint32_t Limit)
{
    PHYSICS_UNUSED(Limit);

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
