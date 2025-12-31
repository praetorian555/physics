#pragma once

#include "physics/core.hpp"
#include "physics/shape.hpp"

namespace Physics
{

struct Body
{
    Vector3r position;
    Quatr orientation = Quatr::Identity();
    Vector3r linear_velocity = Vector3r::Zero();
    Vector3r angular_velocity = Vector3r::Zero();
    real inverse_mass;
    real elasticity = 1.0f;
    real friction;
    Shape* shape;

    [[nodiscard]] Vector3r GetCenterOfMassWorldSpace() const;
    [[nodiscard]] Vector3r GetCenterOfMassBodySpace() const;

    [[nodiscard]] Vector3r WorldSpaceToBodySpace(const Vector3r& world_point) const;
    [[nodiscard]] Vector3r BodySpaceToWorldSpace(const Vector3r& body_point) const;

    [[nodiscard]] Matrix3x3r GetInverseInertiaTensorWorldSpace() const;
    [[nodiscard]] Matrix3x3r GetInverseInertiaTensorBodySpace() const;

    void ApplyImpulseLinear(const Vector3r& impulse);
    void ApplyImpulseAngular(const Vector3r& impulse);
    void ApplyImpulse(const Vector3r& impulse, const Vector3r& world_point);

    void Update(real delta_seconds);

private:
    void LimitAngularVelocity();
};

}  // namespace Physics
