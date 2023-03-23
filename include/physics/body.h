#pragma once

#include "math/quaternion.h"
#include "math/transform.h"

#include "physics/base.h"

namespace physics
{

/**
 * Represents rigid bodies used in physics simulation.
 */
class RigidBody
{
public:
    void SetMass(real mass);
    void SetInverseMass(real inverse_mass);
    [[nodiscard]] real GetMass() const;
    [[nodiscard]] real GetInverseMass() const;

    [[nodiscard]] bool HasFiniteMass() const;

    /**
     * Sets the inertia tensor of the body in the body space.
     * @param inertia_tensor Inertia tensor of the body in the body space.
     */
    void SetInertiaTensor(const math::Transform& inertia_tensor);
    void SetInverseInertiaTensor(const math::Transform& inverse_inertia_tensor);

    /**
     * Gets the inertia tensor of the body in the body space.
     * @return Returns the inertia tensor of the body in the body space.
     */
    [[nodiscard]] math::Transform GetInertiaTensor() const;
    [[nodiscard]] math::Transform GetInverseInertiaTensor() const;

    void SetDamping(real damping);
    [[nodiscard]] real GetDamping() const;
    void SetAngularDamping(real angular_damping);
    [[nodiscard]] real GetAngularDamping() const;

    void SetPosition(const math::Point3& position);
    [[nodiscard]] const math::Point3& GetPosition() const;

    void SetOrientation(const math::Quaternion& orientation);
    [[nodiscard]] const math::Quaternion& GetOrientation() const;

    void SetVelocity(const math::Vector3& velocity);
    [[nodiscard]] const math::Vector3& GetVelocity() const;

    void SetAngularVelocity(const math::Vector3& angular_velocity);
    [[nodiscard]] const math::Vector3& GetAngularVelocity() const;

    /**
     * Sets the linear acceleration of the rigid body in the world space. This will be added
     * to the acceleration by the forces during the integration step.
     * @param acceleration Linear acceleration of the rigid body in the world space.
     */
    void SetAcceleration(const math::Vector3& acceleration);
    [[nodiscard]] const math::Vector3& GetAcceleration() const;

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
     * @param force Force to apply in world space.
     */
    void AddForce(const math::Vector3& force);

    /**
     * Applies the force to the rigid body at the given point.
     * @param force Force to apply in world space.
     * @param point Point of application in world space.
     */
    void AddForceAtPoint(const math::Vector3& force, const math::Point3& point);

    /**
     * Applies the force to the rigid body at the given point.
     * @param force Force to apply in world space.
     * @param point Point of application in body space.
     */
    void AddForceAtLocalPoint(const math::Vector3& force, const math::Point3& point);

    [[nodiscard]] const math::Vector3& GetAccumulatedForce() const;
    [[nodiscard]] const math::Vector3& GetAccumulatedTorque() const;

    /**
     * Updates the rigid body's position, orientation, velocity and angular velocity based on the
     * accumulated forces and torques.
     * @param delta_seconds Time step.
     */
    void Integrate(real delta_seconds);

protected:
    /** Holds linear position of the rigid body in the world space. */
    math::Point3 m_position;

    /** Holds orientation of the rigid body in the world space. */
    math::Quaternion m_orientation;

    /** Holds linear velocity of the rigid body in the world space. */
    math::Vector3 m_velocity;

    /** Holds angular velocity of the rigid body in the world space. */
    math::Vector3 m_angular_velocity;

    /**
     * Holds linear acceleration of the rigid body in the world space set by the user. Not a result
     * of force accumulator.
     */
    math::Vector3 m_acceleration;

    /**
     * Represents value 1 / mass since we often need objects with infinite mass (immovable objects)
     * which then have inverse mass of 0.
     */
    real m_inverse_mass = PHYSICS_REALC(0.0);

    /**
     * Holds the inverse inertia tensor of the rigid body in the body space. It plays the same role
     * for the rotational movement that mass plays for linear motion. The values on the main
     * diagonal represent the inverse inertia of the body along the corresponding local axis. The
     * matrix can't be degenerated (all values on the main diagonal must be different from 0). As
     * long as the tensor is finite, it will be invertible.
     */
    math::Transform m_inverse_inertia_tensor_local;

    /**
     * Simple representation of the drag force used to avoid numerical inaccuracies that can lead
     * to objects accelerating of their own accord.
     */
    real m_damping = PHYSICS_REALC(0.99);

    /**
     * Simple representation of the angular drag force used to avoid numerical inaccuracies that
     * can lead to objects accelerating of their own accord.
     */
    real m_angular_damping = PHYSICS_REALC(0.99);

    /**
     * Holds transform used to move from body space to world space. Cached value calculated
     * based on the position and orientation.
     */
    math::Transform m_to_world_space;

    /**
     * Inverse inertia tensor of the rigid body in the world space. Cached value calculated based on
     * the inverse inertia tensor in the body space and the orientation of the rigid body.
     */
    math::Transform m_inverse_inertia_tensor_world;

    math::Vector3 m_force_accumulator;
    math::Vector3 m_torque_accumulator;
};

}  // namespace physics
