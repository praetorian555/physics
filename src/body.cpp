#include "physics/body.h"

void Physics::RigidBody::SetMass(Physics::real mass)
{
    PHYSICS_ASSERT(mass > MATH_REALC(0.0));
    m_inverse_mass = MATH_REALC(1.0) / mass;
}

void Physics::RigidBody::SetInverseMass(Physics::real inverse_mass)
{
    m_inverse_mass = inverse_mass;
}

Physics::real Physics::RigidBody::GetMass() const
{
    if (m_inverse_mass == MATH_REALC(0.0))
    {
        return math::kInfinity;
    }
    return MATH_REALC(1.0) / m_inverse_mass;
}

Physics::real Physics::RigidBody::GetInverseMass() const
{
    return m_inverse_mass;
}

bool Physics::RigidBody::HasFiniteMass() const
{
    return m_inverse_mass != MATH_REALC(0.0);
}

void Physics::RigidBody::SetInertiaTensor(const math::Transform& inertia_tensor)
{
    m_inverse_inertia_tensor_local = {inertia_tensor.GetInverse(), inertia_tensor.GetMatrix()};
}

void Physics::RigidBody::SetInverseInertiaTensor(const math::Transform& inverse_inertia_tensor)
{
    m_inverse_inertia_tensor_local = inverse_inertia_tensor;
}

math::Transform Physics::RigidBody::GetInertiaTensor() const
{
    return {m_inverse_inertia_tensor_local.GetInverse(),
            m_inverse_inertia_tensor_local.GetMatrix()};
}

math::Transform Physics::RigidBody::GetInverseInertiaTensor() const
{
    return m_inverse_inertia_tensor_local;
}

void Physics::RigidBody::SetDamping(Physics::real damping)
{
    m_damping = damping;
}

Physics::real Physics::RigidBody::GetDamping() const
{
    return m_damping;
}

Physics::real Physics::RigidBody::GetAngularDamping() const
{
    return m_angular_damping;
}

void Physics::RigidBody::SetAngularDamping(Physics::real angular_damping)
{
    m_angular_damping = angular_damping;
}

void Physics::RigidBody::SetPosition(const math::Point3& position)
{
    m_position = position;
}

const math::Point3& Physics::RigidBody::GetPosition() const
{
    return m_position;
}

void Physics::RigidBody::SetOrientation(const math::Quaternion& orientation)
{
    m_orientation = orientation;
}

const math::Quaternion& Physics::RigidBody::GetOrientation() const
{
    return m_orientation;
}

void Physics::RigidBody::SetAngularVelocity(const math::Vector3& angular_velocity)
{
    m_angular_velocity = angular_velocity;
}

const math::Vector3& Physics::RigidBody::GetAngularVelocity() const
{
    return m_angular_velocity;
}

void Physics::RigidBody::SetVelocity(const math::Vector3& velocity)
{
    m_velocity = velocity;
}

const math::Vector3& Physics::RigidBody::GetVelocity() const
{
    return m_velocity;
}

void Physics::RigidBody::SetAcceleration(const math::Vector3& acceleration)
{
    m_acceleration = acceleration;
}

const math::Vector3& Physics::RigidBody::GetAcceleration() const
{
    return m_acceleration;
}

const math::Transform& Physics::RigidBody::GetTransform() const
{
    return m_to_world_space;
}

const math::Transform& Physics::RigidBody::GetInverseInertiaTensorWorld() const
{
    return m_inverse_inertia_tensor_world;
}

void Physics::RigidBody::CalculateDerivedData()
{
    m_to_world_space = math::Translate(m_position) * math::Rotate(m_orientation);
    m_inverse_inertia_tensor_world = math::Rotate(m_orientation) * m_inverse_inertia_tensor_local;
}

void Physics::RigidBody::ClearAccumulators()
{
    m_force_accumulator = math::Vector3::Zero;
    m_torque_accumulator = math::Vector3::Zero;
}

void Physics::RigidBody::AddForce(const math::Vector3& force)
{
    m_force_accumulator += force;
}

void Physics::RigidBody::AddForceAtPoint(const math::Vector3& force, const math::Point3& point)
{
    const math::Vector3 point_vector = point - m_position;
    m_force_accumulator += force;
    m_torque_accumulator = math::Cross(point_vector, force);
}

void Physics::RigidBody::AddForceAtLocalPoint(const math::Vector3& force, const math::Point3& point)
{
    const math::Point3 world_point = m_to_world_space(point);
    AddForceAtPoint(force, world_point);
}

const math::Vector3& Physics::RigidBody::GetAccumulatedForce() const
{
    return m_force_accumulator;
}

const math::Vector3& Physics::RigidBody::GetAccumulatedTorque() const
{
    return m_torque_accumulator;
}

void Physics::RigidBody::Integrate(Physics::real delta_seconds)
{
    math::Vector3 total_acceleration = m_acceleration;
    total_acceleration += m_force_accumulator * m_inverse_mass;

    const math::Vector3 angular_acceleration = m_inverse_inertia_tensor_world(m_torque_accumulator);

    m_velocity += total_acceleration * delta_seconds;
    m_angular_velocity += angular_acceleration * delta_seconds;

    // Apply drag.
    m_velocity *= math::Power(m_damping, delta_seconds);
    m_angular_velocity *= math::Power(m_angular_damping, delta_seconds);

    m_position += m_velocity * delta_seconds;
    const math::Quaternion angular_velocity_quaternion(m_angular_velocity.X, m_angular_velocity.Y,
                                                     m_angular_velocity.Z, PHYSICS_REALC(0.0));
    m_orientation +=
        angular_velocity_quaternion * m_orientation * (PHYSICS_REALC(0.5) * delta_seconds);

    CalculateDerivedData();
    ClearAccumulators();
}
