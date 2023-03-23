#include "physics/body.h"

void physics::RigidBody::SetMass(physics::real mass)
{
    PHYSICS_ASSERT(mass > MATH_REALC(0.0));
    m_inverse_mass = MATH_REALC(1.0) / mass;
}

void physics::RigidBody::SetInverseMass(physics::real inverse_mass)
{
    m_inverse_mass = inverse_mass;
}

physics::real physics::RigidBody::GetMass() const
{
    if (m_inverse_mass == MATH_REALC(0.0))
    {
        return math::kInfinity;
    }
    return MATH_REALC(1.0) / m_inverse_mass;
}

physics::real physics::RigidBody::GetInverseMass() const
{
    return m_inverse_mass;
}

bool physics::RigidBody::HasFiniteMass() const
{
    return m_inverse_mass != MATH_REALC(0.0);
}

void physics::RigidBody::SetInertiaTensor(const math::Transform& inertia_tensor)
{
    m_inverse_inertia_tensor_local = {inertia_tensor.GetInverse(), inertia_tensor.GetMatrix()};
}

void physics::RigidBody::SetInverseInertiaTensor(const math::Transform& inverse_inertia_tensor)
{
    m_inverse_inertia_tensor_local = inverse_inertia_tensor;
}

math::Transform physics::RigidBody::GetInertiaTensor() const
{
    return {m_inverse_inertia_tensor_local.GetInverse(),
            m_inverse_inertia_tensor_local.GetMatrix()};
}

math::Transform physics::RigidBody::GetInverseInertiaTensor() const
{
    return m_inverse_inertia_tensor_local;
}

void physics::RigidBody::SetDamping(physics::real damping)
{
    m_damping = damping;
}

physics::real physics::RigidBody::GetDamping() const
{
    return m_damping;
}

physics::real physics::RigidBody::GetAngularDamping() const
{
    return m_angular_damping;
}

void physics::RigidBody::SetAngularDamping(physics::real angular_damping)
{
    m_angular_damping = angular_damping;
}

void physics::RigidBody::SetPosition(const math::Point3& position)
{
    m_position = position;
}

const math::Point3& physics::RigidBody::GetPosition() const
{
    return m_position;
}

void physics::RigidBody::SetOrientation(const math::Quaternion& orientation)
{
    m_orientation = orientation;
}

const math::Quaternion& physics::RigidBody::GetOrientation() const
{
    return m_orientation;
}

void physics::RigidBody::SetAngularVelocity(const math::Vector3& angular_velocity)
{
    m_angular_velocity = angular_velocity;
}

const math::Vector3& physics::RigidBody::GetAngularVelocity() const
{
    return m_angular_velocity;
}

void physics::RigidBody::SetVelocity(const math::Vector3& velocity)
{
    m_velocity = velocity;
}

const math::Vector3& physics::RigidBody::GetVelocity() const
{
    return m_velocity;
}

void physics::RigidBody::SetAcceleration(const math::Vector3& acceleration)
{
    m_acceleration = acceleration;
}

const math::Vector3& physics::RigidBody::GetAcceleration() const
{
    return m_acceleration;
}

const math::Transform& physics::RigidBody::GetTransform() const
{
    return m_to_world_space;
}

const math::Transform& physics::RigidBody::GetInverseInertiaTensorWorld() const
{
    return m_inverse_inertia_tensor_world;
}

void physics::RigidBody::CalculateDerivedData()
{
    m_to_world_space = math::Translate(m_position) * math::Rotate(m_orientation);
    m_inverse_inertia_tensor_world = math::Rotate(m_orientation) * m_inverse_inertia_tensor_local;
}

void physics::RigidBody::ClearAccumulators()
{
    m_force_accumulator = math::Vector3::Zero;
    m_torque_accumulator = math::Vector3::Zero;
}

void physics::RigidBody::AddForce(const math::Vector3& force)
{
    m_force_accumulator += force;
}

void physics::RigidBody::AddForceAtPoint(const math::Vector3& force, const math::Point3& point)
{
    const math::Vector3 point_vector = point - m_position;
    m_force_accumulator += force;
    m_torque_accumulator = math::Cross(point_vector, force);
}

void physics::RigidBody::AddForceAtLocalPoint(const math::Vector3& force, const math::Point3& point)
{
    const math::Point3 world_point = m_to_world_space(point);
    AddForceAtPoint(force, world_point);
}

const math::Vector3& physics::RigidBody::GetAccumulatedForce() const
{
    return m_force_accumulator;
}

const math::Vector3& physics::RigidBody::GetAccumulatedTorque() const
{
    return m_torque_accumulator;
}

void physics::RigidBody::Integrate(physics::real delta_seconds)
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
