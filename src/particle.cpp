#include "physics/particle.h"

#include <cassert>
#include <cmath>

// TODO(Marko): Move this to math library
#ifdef PHYSICS_REAL_AS_DOUBLE
#define PHYSICS_POW(A, B) std::pow(A, B)
#else
#define PHYSICS_POW(A, B) std::powf(A, B)
#endif

void physics::Particle::Integrate(real DeltaSeconds)
{
    if (DeltaSeconds <= PHYSICS_REALC(0.0))
    {
        return;
    }

    // Immovable objects don't move
    if (m_InverseMass == PHYSICS_REALC(0.0))
    {
        return;
    }

    // Update the position based on the velocity during the previous frame.
    // We are ignoring the factor that depends on the acceleration since the values of acceleration
    // are usually small and additionally multipled by square of time will result in negligable
    // position change.
    m_Position += m_Velocity * DeltaSeconds;

    // TODO(Marko): Calculate the acceleration based on the forces applied to the particle

    // Update the velocity based on the acceleration.
    m_Velocity += m_Acceleration * DeltaSeconds;

    // Apply drag force.
    m_Velocity *= PHYSICS_POW(m_Damping, DeltaSeconds);

    // TODO(Marko): Clear the force accumulator
}

void physics::Particle::SetPosition(const math::Point3& Position)
{
    m_Position = Position;
}

math::Point3 physics::Particle::GetPosition() const
{
    return m_Position;
}

void physics::Particle::SetVelocity(const math::Vector3& Velocity)
{
    m_Velocity = Velocity;
}

math::Vector3 physics::Particle::GetVelocity() const
{
    return m_Velocity;
}

void physics::Particle::SetAcceleration(const math::Vector3& Acceleration)
{
    m_Acceleration = Acceleration;
}

math::Vector3 physics::Particle::GetAcceleration() const
{
    return m_Acceleration;
}

void physics::Particle::SetMass(real Mass)
{
    assert(Mass != PHYSICS_REALC(0.0));
    m_InverseMass = PHYSICS_REALC(1.0) / Mass;
}

void physics::Particle::SetInverseMass(real InverseMass)
{
    m_InverseMass = InverseMass;
}

physics::real physics::Particle::GetMass() const
{
    assert(m_InverseMass != PHYSICS_REALC(0.0));
    return PHYSICS_REALC(1.0) / m_InverseMass;
}

physics::real physics::Particle::GetInverseMass() const
{
    return m_InverseMass;
}

void physics::Particle::SetDamping(real Damping)
{
    assert(Damping >= PHYSICS_REALC(0.0) && Damping <= PHYSICS_REALC(1.0));
    m_Damping = Damping;
}

physics::real physics::Particle::GetDamping() const
{
    return m_Damping;
}
