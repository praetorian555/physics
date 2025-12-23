#include "physics/body.hpp"

Physics::Vector3r Physics::Body::GetCenterOfMassWorldSpace() const
{
    PHYSICS_ASSERT(shape != nullptr, "Shape can't be null!");
    const Vector3r body_center_mass = shape->GetCenterOfMass();
    return position + orientation * body_center_mass;
}

Physics::Vector3r Physics::Body::GetCenterOfMassBodySpace() const
{
    PHYSICS_ASSERT(shape != nullptr, "Shape can't be null!");
    return shape->GetCenterOfMass();
}

Physics::Vector3r Physics::Body::WorldSpaceToBodySpace(const Vector3r& world_point) const
{
    const Vector3r after_translation = world_point - GetCenterOfMassWorldSpace();
    return Opal::Inverse(orientation) * after_translation;
}

Physics::Vector3r Physics::Body::BodySpaceToWorldSpace(const Vector3r& body_point) const
{
    return GetCenterOfMassWorldSpace() + orientation * body_point;
}

Physics::Matrix3x3r Physics::Body::GetInverseInertiaTensorWorldSpace() const
{
    const Matrix3x3r inertia_tensor_local = shape->GetInertiaTensor();
    const Matrix3x3r inverse_inertia_tensor_local = inverse_mass * Opal::Inverse(inertia_tensor_local);
    const Matrix3x3r orientation_mat = orientation.ToMatrix3x3();
    return orientation_mat * inverse_inertia_tensor_local * Transpose(orientation_mat);
}

Physics::Matrix3x3r Physics::Body::GetInverseInertiaTensorBodySpace() const
{
    const Matrix3x3r inertia_tensor_local = shape->GetInertiaTensor();
    return inverse_mass * Opal::Inverse(inertia_tensor_local);
}

void Physics::Body::ApplyImpulseLinear(const Vector3r& impulse)
{
    if (0.0 == inverse_mass)
    {
        return;
    }
    // p = m * v
    // dp = J = m * dv
    // dv = J / m
    linear_velocity += impulse * inverse_mass;
}