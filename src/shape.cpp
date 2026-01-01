#include "physics/shape.hpp"

void Physics::Shape::Build(Opal::ArrayView<Vector3r>) {}

Physics::real Physics::Shape::GetFastestLinearSpeed(const Vector3r&, const Vector3r&) const
{
    return PHYSICS_CONST(0);
}