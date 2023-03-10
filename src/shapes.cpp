#include "physics/shapes.h"

#include <cassert>

physics::AABox::AABox(const math::Vector3& Min, const math::Vector3& Max)
    : Shape(ShapeType::AABox), Min(Min), Max(Max)
{
}

physics::AABox::AABox() : Shape(ShapeType::AABox) {}

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

physics::real physics::AABox::GetVolume() const
{
    const math::Vector3 Diagonal = Max - Min;
    return Diagonal.X * Diagonal.Y * Diagonal.Z;
}

physics::real physics::AABox::GetSurfaceArea() const
{
    const math::Vector3 Diagonal = Max - Min;
    return 2 * Diagonal.X * Diagonal.Y + 2 * Diagonal.X * Diagonal.Z + 2 * Diagonal.Y * Diagonal.Z;
}

physics::Sphere::Sphere(const math::Vector3& Center, physics::real Radius)
    : Shape(ShapeType::Sphere), Center(Center), Radius(Radius)
{
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

physics::real physics::Sphere::GetSurfaceArea() const
{
    constexpr real kConstant = PHYSICS_REALC(4.0) * math::kPi;
    return kConstant * Radius * Radius;
}

physics::real physics::Sphere::GetVolume() const
{
    constexpr real kConstant = PHYSICS_REALC(4.0) / PHYSICS_REALC(3.0) * math::kPi;
    return kConstant * Radius * Radius * Radius;
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
