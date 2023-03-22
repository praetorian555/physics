#pragma once

#include "math/point3.h"

#include "physics/base.h"

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

    bool HasFiniteMass() const;

    void SetDamping(real Damping);
    [[nodiscard]] real GetDamping() const;

    // Adds the given force to the force accumulator. Valid only for the next integration step.
    void AddForce(const math::Vector3& Force);
    [[nodiscard]] const math::Vector3& GetForceAccumulator() const;

    // Clears the force accumulator. Called after each integration step.
    void ClearAccumulator();

protected:
    math::Point3 m_Position;
    math::Vector3 m_Velocity;

    math::Vector3 m_Acceleration;
    uint8_t m_OverrideForces : 1 = 0;

    // Represents value 1 / mass since we often need objects with infinite mass (immovable objects)
    // which then have inverse mass of 0.
    real m_InverseMass = PHYSICS_REALC(0.0);

    // Simple representation of the drag force used to avoid numerical inaccuracies that can lead to
    // objects accelerating of their own accord.
    real m_Damping = PHYSICS_REALC(0.99);

    // Holds the accumulated force to be applied at the next integration step.
    math::Vector3 m_ForceAccumulator;
};

}  // namespace physics
