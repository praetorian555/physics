#pragma once

#include "physics/core.hpp"
#include "physics/shape.hpp"

namespace Physics
{

struct Body
{
    Vector3r position;
    Quatr orientation;
    Vector3r linear_velocity = Vector3r::Zero();
    real inverse_mass;
    Shape* shape;

    [[nodiscard]] Vector3r GetCenterOfMassWorldSpace() const;
    [[nodiscard]] Vector3r GetCenterOfMassBodySpace() const;

    [[nodiscard]] Vector3r WorldSpaceToBodySpace(const Vector3r& world_point) const;
    [[nodiscard]] Vector3r BodySpaceToWorldSpace(const Vector3r& body_point) const;

    void ApplyImpulseLinear(const Vector3r& impulse);
};

}  // namespace Physics
