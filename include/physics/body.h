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

    /**
     * Sets the inertia tensor of the body in the body space.
     * @param InertiaTensor Inertia tensor of the body in the body space.
     */
    void SetInertiaTensor(const math::Transform& InertiaTensor);
    void SetInverseInertiaTensor(const math::Transform& InverseInertiaTensor);

    /**
     * Gets the inertia tensor of the body in the body space.
     * @return Returns the inertia tensor of the body in the body space.
     */
    [[nodiscard]] math::Transform GetInertiaTensor() const;
    [[nodiscard]] math::Transform GetInverseInertiaTensor() const;

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
     * Get the inverse inertia tensor of the rigid body in the world space.
     * @return Returns the inverse inertia tensor.
     */
    [[nodiscard]] const math::Transform& GetInverseInertiaTensorWorld() const;

    /**
     * Calculates derived data from the body's state. This is also called automatically by the
     * integration methods.
     */
    void CalculateDerivedData();

    /**
     * Clears the accumulated forces and torques. Always called at the end of the integration step.
     */
    void ClearAccumulators();

    /**
     * Applies the force to the rigid body's center of mass.
     * @param Force Force to apply in world space.
     */
    void AddForce(const math::Vector3& Force);

    /**
     * Applies the force to the rigid body at the given point.
     * @param Force Force to apply in world space.
     * @param Point Point of application in world space.
     */
    void AddForceAtPoint(const math::Vector3& Force, const math::Point3& Point);

    /**
     * Applies the force to the rigid body at the given point.
     * @param Force Force to apply in world space.
     * @param Point Point of application in body space.
     */
    void AddForceAtLocalPoint(const math::Vector3& Force, const math::Point3& Point);

    const math::Vector3& GetAccumulatedForce() const;
    const math::Vector3& GetAccumulatedTorque() const;

protected:
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
    math::Transform m_InverseInertiaTensorLocal;

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

    /**
     * Inverse inertia tensor of the rigid body in the world space. Cached value calculated based on
     * the inverse inertia tensor in the body space and the orientation of the rigid body.
     */
    math::Transform m_InverseInertiaTensorWorld;

    math::Vector3 m_ForceAccumulator;
    math::Vector3 m_TorqueAccumulator;
};

}  // namespace physics
