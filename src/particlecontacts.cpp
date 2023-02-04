#include "physics/particlecontacts.h"

#include <cassert>

#include "physics/particle.h"

physics::ParticleContact* physics::ParticleContact::Create(physics::Particle* MainParticle,
                                                           physics::Particle* OtherParticle,
                                                           physics::real Restitution)
{
    if (MainParticle == nullptr || OtherParticle == nullptr)
    {
        return nullptr;
    }
    if (Restitution < PHYSICS_REALC(0.0) || Restitution > PHYSICS_REALC(1.0))
    {
        return nullptr;
    }
    assert(!math::IsNaN(Restitution));
    assert(!MainParticle->GetPosition().HasNaNs());
    assert(!OtherParticle->GetPosition().HasNaNs());

    ParticleContact* NewContact = new ParticleContact();
    NewContact->m_MainParticle = MainParticle;
    NewContact->m_OtherParticle = OtherParticle;
    NewContact->m_Restitution = Restitution;
    NewContact->m_ContactNormal = MainParticle->GetPosition() - OtherParticle->GetPosition();
    NewContact->m_ContactNormal = math::Normalize(NewContact->m_ContactNormal);

    return NewContact;
}

physics::ParticleContact* physics::ParticleContact::Create(physics::Particle* MainParticle,
                                                           physics::real Restitution,
                                                           const math::Vector3& ContactNormal)
{
    if (MainParticle == nullptr)
    {
        return nullptr;
    }
    if (Restitution < PHYSICS_REALC(0.0) || Restitution > PHYSICS_REALC(1.0))
    {
        return nullptr;
    }
    if (ContactNormal == math::Vector3::Zero)
    {
        return nullptr;
    }
    assert(!ContactNormal.HasNaNs());

    ParticleContact* NewContact = new ParticleContact();
    NewContact->m_MainParticle = MainParticle;
    NewContact->m_OtherParticle = nullptr;
    NewContact->m_Restitution = Restitution;
    NewContact->m_ContactNormal = math::Normalize(ContactNormal);

    return NewContact;
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

    const real NewSeparatingVelocity = -SeparatingVelocity * m_Restitution;
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
