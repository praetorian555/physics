#pragma once

#include "physics/core.hpp"
#include "physics/shape.hpp"

namespace Physics
{

struct Body
{
    Vector3r position;
    Quatr orientation;
    Shape* shape;

    [[nodiscard]] Vector3r GetCenterOfMassWorldSpace() const;
    [[nodiscard]] Vector3r GetCenterOfMassBodySpace() const;

    [[nodiscard]] Vector3r WorldSpaceToBodySpace(const Vector3r& world_point) const;
    [[nodiscard]] Vector3r BodySpaceToWorldSpace(const Vector3r& body_point) const;
};

}  // namespace Physics
