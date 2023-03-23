#include "physics/particle-force-generator.h"

#include "physics/particle.h"

void physics::ParticleForceRegistry::Add(physics::Particle* particle,
                                         physics::ParticleForceGenerator* force_generator)
{
    m_entries.push_back({particle, force_generator});
}

void physics::ParticleForceRegistry::Remove(physics::Particle* particle,
                                            physics::ParticleForceGenerator* force_generator)
{
    std::erase_if(
        m_entries, [particle, force_generator](const Entry& entry)
                  { return entry.particle == particle && entry.force_generator == force_generator; });
}

void physics::ParticleForceRegistry::Clear()
{
    m_entries.clear();
}

void physics::ParticleForceRegistry::UpdateForces(real delta_seconds)
{
    for (const Entry& entry : m_entries)
    {
        entry.force_generator->UpdateForce(*entry.particle, delta_seconds);
    }
}

void physics::ParticleGravity::UpdateForce(physics::Particle& particle, real delta_seconds)
{
    PHYSICS_UNUSED(delta_seconds);

    if (!particle.HasFiniteMass())
    {
        return;
    }
    particle.AddForce(m_gravity * particle.GetInverseMass());
}

void physics::ParticleDrag::UpdateForce(physics::Particle& particle, real delta_seconds)
{
    PHYSICS_UNUSED(delta_seconds);

    if (!particle.HasFiniteMass())
    {
        return;
    }

    const math::Vector3 velocity = particle.GetVelocity();
    const real drag_coefficient = velocity.Length() * m_k1 + velocity.LengthSquared() * m_k2;
    const math::Vector3 drag_force = Normalize(velocity) * -drag_coefficient;

    particle.AddForce(drag_force);
}

void physics::ParticleSpring::UpdateForce(physics::Particle& particle, real delta_seconds)
{
    PHYSICS_UNUSED(delta_seconds);

    math::Vector3 length_vector = particle.GetPosition() - m_other->GetPosition();
    const real length = length_vector.Length();
    const real spring_force = m_spring_constant * (length - m_rest_length);
    length_vector = math::Normalize(length_vector);

    particle.AddForce(-length_vector * spring_force);
}

void physics::ParticleAnchoredSpring::UpdateForce(physics::Particle& particle, real delta_seconds)
{
    PHYSICS_UNUSED(delta_seconds);

    math::Vector3 length_vector = particle.GetPosition() - m_anchor;
    const real length = length_vector.Length();
    const real spring_force = m_spring_constant * (length - m_rest_length);
    length_vector = math::Normalize(length_vector);

    particle.AddForce(-length_vector * spring_force);
}

void physics::ParticleBungee::UpdateForce(physics::Particle& particle, real delta_seconds)
{
    PHYSICS_UNUSED(delta_seconds);

    math::Vector3 length_vector = particle.GetPosition() - m_other->GetPosition();
    const real length = length_vector.Length();
    if (length <= m_rest_length)
    {
        return;
    }

    const real spring_force = m_spring_constant * (length - m_rest_length);
    length_vector = math::Normalize(length_vector);

    particle.AddForce(-length_vector * spring_force);
}

void physics::ParticleBuoyancy::UpdateForce(physics::Particle& particle, real delta_seconds)
{
    PHYSICS_UNUSED(delta_seconds);

    // Current depth of the particle.
    const real depth = particle.GetPosition().Y;

    // If we are completely out of the water, do nothing.
    if (depth >= m_water_height + m_max_depth)
    {
        return;
    }

    // If we are fully submerged, apply the maximum buoyancy force.
    if (depth <= m_water_height - m_max_depth)
    {
        math::Vector3 force = math::Vector3::Zero;
        force.Y = m_volume * m_liquid_density;
        particle.AddForce(force);
        return;
    }

    const real submerged_amount = (m_max_depth + m_water_height - depth) / (2 * m_max_depth);
    math::Vector3 force = math::Vector3::Zero;
    force.Y = m_volume * m_liquid_density * submerged_amount;
    particle.AddForce(force);
}
