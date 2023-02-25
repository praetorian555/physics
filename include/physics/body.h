#pragma once

#include "math/point3.h"
#include "math/quaternion.h"
#include "math/transform.h"
#include "math/vector3.h"

#include "physics/base.h"

namespace physics
{

/**
 * Represents rigid bodies used in physics simulation.
 */
class RigidBody
{
public:
    void SetMass(real Mass);
    void SetInverseMass(real InverseMass);
    [[nodiscard]] real GetMass() const;
    [[nodiscard]] real GetInverseMass() const;

    void SetInertiaTensor(const math::Matrix4x4& InertiaTensor);
    void SetInverseInertiaTensor(const math::Matrix4x4& InverseInertiaTensor);
    [[nodiscard]] math::Matrix4x4 GetInertiaTensor() const;
    [[nodiscard]] math::Matrix4x4 GetInverseInertiaTensor() const;

    void SetDamping(real Damping);
    [[nodiscard]] real GetDamping() const;

    void SetPosition(const math::Point3& Position);
    [[nodiscard]] const math::Point3& GetPosition() const;

    void SetOrientation(const math::Quaternion& Orientation);
    [[nodiscard]] const math::Quaternion& GetOrientation() const;

    void SetVelocity(const math::Vector3& Velocity);
    [[nodiscard]] const math::Vector3& GetVelocity() const;

    void SetAngularVelocity(const math::Vector3& AngularVelocity);
    [[nodiscard]] const math::Vector3& GetAngularVelocity() const;

    /**
     * Get the transform matrix that transforms from body space to world space.
     * @return Returns the transform matrix.
     */
    [[nodiscard]] const math::Transform& GetTransform() const;

    /**
     * Calculates derived data from the body's state. This is also called automatically by the
     * integration methods.
     */
    void CalculateDerivedData();

protected:
    void CalculateTransformMatrix();

    /** Holds linear position of the rigid body in the world space. */
    math::Point3 m_Position;

    /** Holds orientation of the rigid body in the world space. */
    math::Quaternion m_Orientation;

    /** Holds linear velocity of the rigid body in the world space. */
    math::Vector3 m_Velocity;

    /** Holds angular velocity of the rigid body in the world space. */
    math::Vector3 m_AngularVelocity;

    /**
     * Represents value 1 / mass since we often need objects with infinite mass (immovable objects)
     * which then have inverse mass of 0.
     */
    real m_InverseMass = PHYSICS_REALC(0.0);

    /**
     * Holds the inverse inertia tensor of the rigid body in the body space. It plays the same role
     * for the rotational movement that mass plays for linear motion. The values on the main
     * diagonal represent the inverse inertia of the body along the corresponding local axis. The
     * matrix can't be degenerated (all values on the main diagonal must be different from 0). As
     * long as the tensor is finite, it will be invertible.
     */
    math::Matrix4x4 m_InverseInertiaTensor;

    /**
     * Simple representation of the drag force used to avoid numerical inaccuracies that can lead
     * to objects accelerating of their own accord.
     */
    real m_Damping = PHYSICS_REALC(0.99);

    /**
     * Holds transform used to move from body space to world space. Cached value calculated
     * based on the position and orientation.
     */
    math::Transform m_ToWorldSpace;
};

}  // namespace physics