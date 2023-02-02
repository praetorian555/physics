#include "physics/particleforcegenerator.h"

#include "physics/particle.h"

void physics::ParticleForceRegistry::Add(physics::Particle* Particle,
                                         physics::ParticleForceGenerator* ForceGenerator)
{
    m_Entries.push_back({Particle, ForceGenerator});
}

void physics::ParticleForceRegistry::Remove(physics::Particle* Particle,
                                            physics::ParticleForceGenerator* ForceGenerator)
{
    std::erase_if(m_Entries, [Particle, ForceGenerator](const Entry& Entry)
                  { return Entry.Particle == Particle && Entry.ForceGenerator == ForceGenerator; });
}

void physics::ParticleForceRegistry::Clear()
{
    m_Entries.clear();
}

void physics::ParticleForceRegistry::UpdateForces(float DeltaSeconds)
{
    for (const Entry& Entry : m_Entries)
    {
        Entry.ForceGenerator->UpdateForce(*Entry.Particle, DeltaSeconds);
    }
}

void physics::ParticleGravity::UpdateForce(physics::Particle& Particle, float DeltaSeconds)
{
    PHYSICS_UNUSED(DeltaSeconds);

    if (!Particle.HasFiniteMass())
    {
        return;
    }
    Particle.AddForce(m_Gravity * Particle.GetInverseMass());
}

void physics::ParticleDrag::UpdateForce(physics::Particle& Particle, float DeltaSeconds)
{
    PHYSICS_UNUSED(DeltaSeconds);

    if (!Particle.HasFiniteMass())
    {
        return;
    }

    const math::Vector3 Velocity = Particle.GetVelocity();
    const real DragCoefficient = Velocity.Length() * m_K1 + Velocity.LengthSquared() * m_K2;
    const math::Vector3 DragForce = Normalize(Velocity) * -DragCoefficient;

    Particle.AddForce(DragForce);
}

void physics::ParticleSpring::UpdateForce(physics::Particle& Particle, float DeltaSeconds)
{
    PHYSICS_UNUSED(DeltaSeconds);

    math::Vector3 LengthVector = Particle.GetPosition() - m_Other->GetPosition();
    const real Length = LengthVector.Length();
    const real SpringForce = m_SpringConstant * math::Abs(Length - m_RestLength);
    LengthVector = math::Normalize(LengthVector);

    Particle.AddForce(-LengthVector * SpringForce);
}

void physics::ParticleAnchoredSpring::UpdateForce(physics::Particle& Particle, float DeltaSeconds)
{
    PHYSICS_UNUSED(DeltaSeconds);

    math::Vector3 LengthVector = Particle.GetPosition() - m_Anchor;
    const real Length = LengthVector.Length();
    const real SpringForce = m_SpringConstant * math::Abs(Length - m_RestLength);
    LengthVector = math::Normalize(LengthVector);

    Particle.AddForce(-LengthVector * SpringForce);
}

void physics::ParticleBungee::UpdateForce(physics::Particle& Particle, float DeltaSeconds)
{
    PHYSICS_UNUSED(DeltaSeconds);

    math::Vector3 LengthVector = Particle.GetPosition() - m_Other->GetPosition();
    const real Length = LengthVector.Length();
    if (Length <= m_RestLength)
    {
        return;
    }

    const real SpringForce = m_SpringConstant * math::Abs(Length - m_RestLength);
    LengthVector = math::Normalize(LengthVector);

    Particle.AddForce(-LengthVector * SpringForce);
}

void physics::ParticleBuoyancy::UpdateForce(physics::Particle& Particle, float DeltaSeconds)
{
    PHYSICS_UNUSED(DeltaSeconds);

    // Current depth of the particle.
    const real Depth = Particle.GetPosition().Y;

    // If we are completely out of the water, do nothing.
    if (Depth >= m_WaterHeight + m_MaxDepth)
    {
        return;
    }

    // If we are fully submerged, apply the maximum buoyancy force.
    if (Depth <= m_WaterHeight - m_MaxDepth)
    {
        math::Vector3 Force = math::Vector3::Zero;
        Force.Y = m_Volume * m_LiquidDensity;
        Particle.AddForce(Force);
        return;
    }

    const real SubmergedAmount = (Depth - m_MaxDepth - m_WaterHeight) / (2 * m_MaxDepth);
    math::Vector3 Force = math::Vector3::Zero;
    Force.Y = m_Volume * m_LiquidDensity * SubmergedAmount;
    Particle.AddForce(Force);
}
