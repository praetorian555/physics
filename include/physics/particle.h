#pragma once

#include "physics/base.h"

#include "math/point3.h"
#include "math/vector3.h"

namespace physics
{

// Used to represent the point mass object in world space.
class Particle
{
public:
    Particle() = default;

    // Updates position and velocity of the particle based on the currently applied forces. Using
    // Newton-Euler method to approximate the integral for position update.
    void Integrate(real DeltaSeconds);

    void SetPosition(const math::Point3& Position);
    [[nodiscard]] math::Point3 GetPosition() const;

    void SetVelocity(const math::Vector3& Velocity);
    [[nodiscard]] math::Vector3 GetVelocity() const;

    void SetAcceleration(const math::Vector3& Acceleration);
    [[nodiscard]] math::Vector3 GetAcceleration() const;

    void SetMass(real Mass);
    void SetInverseMass(real InverseMass);
    [[nodiscard]] real GetMass() const;
    [[nodiscard]] real GetInverseMass() const;

    void SetDamping(real Damping);
    [[nodiscard]] real GetDamping() const;

protected:
    math::Point3 m_Position;
    math::Vector3 m_Velocity;
    math::Vector3 m_Acceleration;

    // Represents value 1 / mass since we often need objects with infinite mass (immovable objects)
    // which then have inverse mass of 0.
    real m_InverseMass = PHYSICS_REALC(0.0);

    // Simple representation of the drag force used to avoid numerical inaccuracies that can lead to
    // objects accelerating of their own accord.
    real m_Damping = PHYSICS_REALC(0.99);
};

}  // namespace physics
