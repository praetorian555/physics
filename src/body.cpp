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
    if (PHYSICS_CONST(0.0) == inverse_mass)
    {
        return;
    }
    // p = m * v
    // dp = J = m * dv
    // dv = J / m
    linear_velocity += impulse * inverse_mass;
}

void Physics::Body::ApplyImpulseAngular(const Vector3r& impulse)
{
    if (PHYSICS_CONST(0.0) == inverse_mass)
    {
        return;
    }
    angular_velocity += GetInverseInertiaTensorWorldSpace() * impulse;
    constexpr real k_max_angular_velocity = PHYSICS_CONST(30.0);
    if (Opal::LengthSquared(angular_velocity) > k_max_angular_velocity * k_max_angular_velocity)
    {
        angular_velocity = Opal::Normalize(angular_velocity) * k_max_angular_velocity;
    }
}

void Physics::Body::ApplyImpulse(const Vector3r& impulse, const Vector3r& world_point)
{
    if (PHYSICS_CONST(0.0) == inverse_mass)
    {
        return;
    }
    ApplyImpulseLinear(impulse);
    const Vector3r com_world = GetCenterOfMassWorldSpace();
    const Vector3r r = world_point - com_world;
    ApplyImpulseAngular(Cross(r, impulse));
}

void Physics::Body::Update(real delta_seconds)
{
    position += linear_velocity * delta_seconds;

    const Vector3r com_world = GetCenterOfMassWorldSpace();
    const Vector3r r = position - com_world;

    // Account for internal torque (precession)
    // T = T_external + omega x I * omega
    // T_external = 0, here it is 0 since we already accounted for in collision response function
    // T = I * alpha = omega x I * omega
    // alpha = I^-1 * (omega x I * omega)
    const Matrix3x3r orientation_mat = orientation.ToMatrix3x3();
    const Matrix3x3r inertia_tensor = orientation_mat * shape->GetInertiaTensor() * Opal::Transpose(orientation_mat);
    const Vector3r angular_acc = Opal::Inverse(inertia_tensor) * Opal::Cross(angular_velocity, inertia_tensor * angular_velocity);
    angular_velocity += angular_acc * delta_seconds;

    // Update orientation
    const Vector3r delta_angle = angular_velocity * delta_seconds;
    const Quatr delta_orientation = Quatr::FromAxisAngleRadians(delta_angle, static_cast<real>(Opal::Length(delta_angle)));
    orientation = delta_orientation * orientation;
    orientation = Opal::Normalize(orientation);

    // Update position
    // TODO: From what I see in textbooks precession should not affect the position of the center of mass
    //position = com_world + delta_orientation * r;
}
