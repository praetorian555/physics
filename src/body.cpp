#include "physics/body.h"

#include <cassert>

void physics::RigidBody::SetMass(physics::real Mass)
{
    assert(Mass > MATH_REALC(0.0));
    m_InverseMass = MATH_REALC(1.0) / Mass;
}

void physics::RigidBody::SetInverseMass(physics::real InverseMass)
{
    m_InverseMass = InverseMass;
}

physics::real physics::RigidBody::GetMass() const
{
    if (m_InverseMass == MATH_REALC(0.0))
    {
        return math::kInfinity;
    }
    return MATH_REALC(1.0) / m_InverseMass;
}

physics::real physics::RigidBody::GetInverseMass() const
{
    return m_InverseMass;
}

bool physics::RigidBody::HasFiniteMass() const
{
    return m_InverseMass != MATH_REALC(0.0);
}

void physics::RigidBody::SetInertiaTensor(const math::Transform& InertiaTensor)
{
    m_InverseInertiaTensorLocal = {InertiaTensor.GetInverse(), InertiaTensor.GetMatrix()};
}

void physics::RigidBody::SetInverseInertiaTensor(const math::Transform& InverseInertiaTensor)
{
    m_InverseInertiaTensorLocal = InverseInertiaTensor;
}

math::Transform physics::RigidBody::GetInertiaTensor() const
{
    return {m_InverseInertiaTensorLocal.GetInverse(), m_InverseInertiaTensorLocal.GetMatrix()};
}

math::Transform physics::RigidBody::GetInverseInertiaTensor() const
{
    return m_InverseInertiaTensorLocal;
}

void physics::RigidBody::SetDamping(physics::real Damping)
{
    m_Damping = Damping;
}

physics::real physics::RigidBody::GetDamping() const
{
    return m_Damping;
}

void physics::RigidBody::SetPosition(const math::Point3& Position)
{
    m_Position = Position;
}

const math::Point3& physics::RigidBody::GetPosition() const
{
    return m_Position;
}

void physics::RigidBody::SetOrientation(const math::Quaternion& Orientation)
{
    m_Orientation = Orientation;
}

const math::Quaternion& physics::RigidBody::GetOrientation() const
{
    return m_Orientation;
}

void physics::RigidBody::SetAngularVelocity(const math::Vector3& AngularVelocity)
{
    m_AngularVelocity = AngularVelocity;
}

const math::Vector3& physics::RigidBody::GetAngularVelocity() const
{
    return m_AngularVelocity;
}

void physics::RigidBody::SetVelocity(const math::Vector3& Velocity)
{
    m_Velocity = Velocity;
}

const math::Vector3& physics::RigidBody::GetVelocity() const
{
    return m_Velocity;
}

const math::Transform& physics::RigidBody::GetTransform() const
{
    return m_ToWorldSpace;
}

const math::Transform& physics::RigidBody::GetInverseInertiaTensorWorld() const
{
    return m_InverseInertiaTensorWorld;
}

void physics::RigidBody::CalculateDerivedData()
{
    m_ToWorldSpace = math::Translate(m_Position) * math::Rotate(m_Orientation);
    m_InverseInertiaTensorWorld = math::Rotate(m_Orientation) * m_InverseInertiaTensorLocal;
}

void physics::RigidBody::ClearAccumulators()
{
    m_ForceAccumulator = math::Vector3::Zero;
    m_TorqueAccumulator = math::Vector3::Zero;
}

void physics::RigidBody::AddForce(const math::Vector3& Force)
{
    m_ForceAccumulator += Force;
}

void physics::RigidBody::AddForceAtPoint(const math::Vector3& Force, const math::Point3& Point)
{
    const math::Vector3 PointVector = Point - m_Position;
    m_ForceAccumulator += Force;
    m_TorqueAccumulator = math::Cross(PointVector, Force);
}

void physics::RigidBody::AddForceAtLocalPoint(const math::Vector3& Force, const math::Point3& Point)
{
    const math::Point3 WorldPoint = m_ToWorldSpace(Point);
    AddForceAtPoint(Force, WorldPoint);
}

const math::Vector3& physics::RigidBody::GetAccumulatedForce() const
{
    return m_ForceAccumulator;
}

const math::Vector3& physics::RigidBody::GetAccumulatedTorque() const
{
    return m_TorqueAccumulator;
}

physics::real physics::RigidBody::GetAngularDamping() const
{
    return m_AngularDamping;
}

void physics::RigidBody::SetAngularDamping(physics::real AngularDamping)
{
    m_AngularDamping = AngularDamping;
}
