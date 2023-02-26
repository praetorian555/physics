#pragma once

#include "math/vector3.h"

#include "physics/base.h"

namespace physics
{

class RigidBody;

/**
 * Represents a force generator interface used to generate forces and apply them to rigid bodies.
 */
class ForceGenerator
{
public:
    virtual ~ForceGenerator() = default;

    /**
     * Calculates and applies the force to the given rigid body.
     * @param Body Rigid body to apply the force to.
     * @param DeltaSeconds Time since last frame.
     */
    virtual void UpdateForce(RigidBody& Body, real DeltaSeconds) = 0;
};

/**
 * Represents a force generator that applies gravity force to the rigid body. Can be used on
 * multiple rigid bodies.
 */
class Gravity : public ForceGenerator
{
public:
    explicit Gravity(const math::Vector3& Gravity) : m_Gravity(Gravity) {}

    void UpdateForce(RigidBody& Body, real DeltaSeconds) override;

    void SetGravity(const math::Vector3& Gravity) { m_Gravity = Gravity; }
    [[nodiscard]] const math::Vector3& GetGravity() const { return m_Gravity; }

private:
    math::Vector3 m_Gravity;
};

}  // namespace physics
