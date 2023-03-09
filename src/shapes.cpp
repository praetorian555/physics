#include "physics/shapes.h"

#include <cassert>

bool physics::AABox::Overlaps(const Shape& Other) const
{
    switch (Other.Type)
    {
        case ShapeType::AABox:
            return ::physics::Overlaps(*this, static_cast<const AABox&>(Other));
        case ShapeType::Sphere:
            return ::physics::Overlaps(*this, static_cast<const Sphere&>(Other));
        default:
            assert(false);
    }
    return false;
}

bool physics::Sphere::Overlaps(const physics::Shape& Other) const
{
    switch (Other.Type)
    {
        case ShapeType::Sphere:
            return ::physics::Overlaps(*this, static_cast<const Sphere&>(Other));
        case ShapeType::AABox:
            return ::physics::Overlaps(*this, static_cast<const AABox&>(Other));
        default:
            assert(false);
    }
    return false;
}

bool physics::Overlaps(const AABox& A, const AABox& B)
{
    if (A.Max.X < B.Min.X || A.Min.X > B.Max.X)
    {
        return false;
    }
    if (A.Max.Y < B.Min.Y || A.Min.Y > B.Max.Y)
    {
        return false;
    }
    if (A.Max.Z < B.Min.Z || A.Min.Z > B.Max.Z)
    {
        return false;
    }
    return true;
}

bool physics::Overlaps(const Sphere& A, const Sphere& B)
{
    const math::Vector3 CenterDiff = A.Center - B.Center;
    const real RadiusSum = A.Radius + B.Radius;
    return CenterDiff.LengthSquared() <= RadiusSum * RadiusSum;
}

bool physics::Overlaps(const AABox& A, const Sphere& B)
{
    // TODO: Implement
    PHYSICS_UNUSED(A);
    PHYSICS_UNUSED(B);
    assert(false && "Not implemented");
    return false;
}

bool physics::Overlaps(const Sphere& A, const AABox& B)
{
    return Overlaps(B, A);
}
