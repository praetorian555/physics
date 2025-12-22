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