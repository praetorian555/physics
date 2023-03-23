#pragma once

#include "physics/base.h"

namespace physics
{

/**
 * Used to represent the point mass object in world space.
 */
class Particle
{
public:
    Particle() = default;

    /**
     * Updates position and velocity of the particle based on the currently applied forces. Using
     * Newton-Euler method to approximate the integral for position update.
     */
    void Integrate(real delta_seconds);

    void SetPosition(const math::Point3& position);
    [[nodiscard]] math::Point3 GetPosition() const;

    void SetVelocity(const math::Vector3& velocity);
    [[nodiscard]] math::Vector3 GetVelocity() const;

    void SetAcceleration(const math::Vector3& acceleration);
    [[nodiscard]] math::Vector3 GetAcceleration() const;

    void SetMass(real mass);
    void SetInverseMass(real inverse_mass);
    [[nodiscard]] real GetMass() const;
    [[nodiscard]] real GetInverseMass() const;

    [[nodiscard]] bool HasFiniteMass() const;

    void SetDamping(real damping);
    [[nodiscard]] real GetDamping() const;

    /**
     * Adds the given force to the force accumulator. The force will be applied at the next
     * integration step.
     * @param force - The force to add.
     */
    void AddForce(const math::Vector3& force);
    [[nodiscard]] const math::Vector3& GetForceAccumulator() const;

    /** Clears the force accumulator. Called after each integration step. */
    void ClearAccumulator();

protected:
    math::Point3 m_position;
    math::Vector3 m_velocity;

    math::Vector3 m_acceleration;
    uint8_t m_override_forces : 1 = 0;

    /**
     * Represents value 1 / mass since we often need objects with infinite mass (immovable objects)
     * which then have inverse mass of 0.
     */
    real m_inverse_mass = PHYSICS_REALC(0.0);

    /**
     * Simple representation of the drag force used to avoid numerical inaccuracies that can lead to
     * objects accelerating of their own accord.
     */
    real m_damping = PHYSICS_REALC(0.99);

    /** Holds the accumulated force to be applied at the next integration step. */
    math::Vector3 m_force_accumulator;
};

}  // namespace physics
