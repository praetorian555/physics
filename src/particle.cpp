#include "physics/particle.h"

void Physics::Particle::Integrate(real delta_seconds)
{
    if (delta_seconds <= PHYSICS_REALC(0.0))
    {
        return;
    }

    // Immovable objects don't move
    if (m_inverse_mass == PHYSICS_REALC(0.0))
    {
        return;
    }

    // Update the position based on the velocity during the previous frame.
    // We are ignoring the factor that depends on the acceleration since the values of acceleration
    // are usually small and additionally multiply by square of time will result in negligible
    // position change.
    m_position += m_velocity * delta_seconds;

    if (!m_override_forces)
    {
        m_acceleration = m_force_accumulator * m_inverse_mass;
    }

    // Update the velocity based on the acceleration.
    m_velocity += m_acceleration * delta_seconds;

    // Apply drag force.
    m_velocity *= math::Power(m_damping, delta_seconds);

    ClearAccumulator();
    m_override_forces = false;
}

void Physics::Particle::SetPosition(const math::Point3& position)
{
    m_position = position;
}

math::Point3 Physics::Particle::GetPosition() const
{
    return m_position;
}

void Physics::Particle::SetVelocity(const math::Vector3& velocity)
{
    m_velocity = velocity;
}

math::Vector3 Physics::Particle::GetVelocity() const
{
    return m_velocity;
}

void Physics::Particle::SetAcceleration(const math::Vector3& acceleration)
{
    m_acceleration = acceleration;
    m_override_forces = true;
}

math::Vector3 Physics::Particle::GetAcceleration() const
{
    return m_acceleration;
}

void Physics::Particle::SetMass(real mass)
{
    PHYSICS_ASSERT(mass != PHYSICS_REALC(0.0));
    m_inverse_mass = PHYSICS_REALC(1.0) / mass;
}

void Physics::Particle::SetInverseMass(real inverse_mass)
{
    m_inverse_mass = inverse_mass;
}

Physics::real Physics::Particle::GetMass() const
{
    PHYSICS_ASSERT(m_inverse_mass != PHYSICS_REALC(0.0));
    return PHYSICS_REALC(1.0) / m_inverse_mass;
}

Physics::real Physics::Particle::GetInverseMass() const
{
    return m_inverse_mass;
}

bool Physics::Particle::HasFiniteMass() const
{
    return m_inverse_mass != PHYSICS_REALC(0.0);
}

void Physics::Particle::SetDamping(real damping)
{
    PHYSICS_ASSERT(damping >= PHYSICS_REALC(0.0) && damping <= PHYSICS_REALC(1.0));
    m_damping = damping;
}

Physics::real Physics::Particle::GetDamping() const
{
    return m_damping;
}

void Physics::Particle::ClearAccumulator()
{
    m_force_accumulator = math::Vector3{};
}

void Physics::Particle::AddForce(const math::Vector3& force)
{
    m_force_accumulator += force;
}

const math::Vector3& Physics::Particle::GetForceAccumulator() const
{
    return m_force_accumulator;
}
