#include "physics/particle.h"

void physics::Particle::Integrate(real delta_seconds)
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

void physics::Particle::SetPosition(const math::Point3& position)
{
    m_position = position;
}

math::Point3 physics::Particle::GetPosition() const
{
    return m_position;
}

void physics::Particle::SetVelocity(const math::Vector3& velocity)
{
    m_velocity = velocity;
}

math::Vector3 physics::Particle::GetVelocity() const
{
    return m_velocity;
}

void physics::Particle::SetAcceleration(const math::Vector3& acceleration)
{
    m_acceleration = acceleration;
    m_override_forces = true;
}

math::Vector3 physics::Particle::GetAcceleration() const
{
    return m_acceleration;
}

void physics::Particle::SetMass(real mass)
{
    assert(mass != PHYSICS_REALC(0.0));
    m_inverse_mass = PHYSICS_REALC(1.0) / mass;
}

void physics::Particle::SetInverseMass(real inverse_mass)
{
    m_inverse_mass = inverse_mass;
}

physics::real physics::Particle::GetMass() const
{
    assert(m_inverse_mass != PHYSICS_REALC(0.0));
    return PHYSICS_REALC(1.0) / m_inverse_mass;
}

physics::real physics::Particle::GetInverseMass() const
{
    return m_inverse_mass;
}

bool physics::Particle::HasFiniteMass() const
{
    return m_inverse_mass != PHYSICS_REALC(0.0);
}

void physics::Particle::SetDamping(real damping)
{
    assert(damping >= PHYSICS_REALC(0.0) && damping <= PHYSICS_REALC(1.0));
    m_damping = damping;
}

physics::real physics::Particle::GetDamping() const
{
    return m_damping;
}

void physics::Particle::ClearAccumulator()
{
    m_force_accumulator = math::Vector3{};
}

void physics::Particle::AddForce(const math::Vector3& force)
{
    m_force_accumulator += force;
}

const math::Vector3& physics::Particle::GetForceAccumulator() const
{
    return m_force_accumulator;
}
