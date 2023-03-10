#include "physics/shapes.h"

#include <cassert>

physics::AABox::AABox(const math::Vector3& Min, const math::Vector3& Max)
    : Shape(ShapeType::AABox), Min(Min), Max(Max)
{
}

physics::AABox::AABox() : Shape(ShapeType::AABox) {}

bool physics::AABox::IsValid() const
{
    return Min != Max;
}

bool physics::AABox::Overlaps(const Shape& Other) const
{
    switch (Other.Type)
    {
        case ShapeType::AABox:
            return ::physics::Overlaps(*this, static_cast<const AABox&>(Other));
        case ShapeType::Sphere:
            return ::physics::Overlaps(*this, static_cast<const Sphere&>(Other));
        case ShapeType::Plane:
            return ::physics::Overlaps(*this, static_cast<const Plane&>(Other));
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

physics::Sphere::Sphere() : Shape(ShapeType::Sphere), Radius(PHYSICS_REALC(0.0)) {}

physics::Sphere::Sphere(const math::Vector3& Center, physics::real Radius)
    : Shape(ShapeType::Sphere), Center(Center), Radius(Radius)
{
}

bool physics::Sphere::IsValid() const
{
    return Radius > PHYSICS_REALC(0.0);
}

bool physics::Sphere::Overlaps(const physics::Shape& Other) const
{
    switch (Other.Type)
    {
        case ShapeType::Sphere:
            return ::physics::Overlaps(*this, static_cast<const Sphere&>(Other));
        case ShapeType::AABox:
            return ::physics::Overlaps(*this, static_cast<const AABox&>(Other));
        case ShapeType::Plane:
            return ::physics::Overlaps(*this, static_cast<const Plane&>(Other));
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

physics::Plane::Plane() : Shape(ShapeType::Plane), Distance(PHYSICS_REALC(0.0)) {}

physics::Plane::Plane(const math::Vector3& Normal, physics::real Distance)
    : Shape(ShapeType::Plane), Normal(Normal), Distance(Distance)
{
    if (this->Normal != math::Vector3::Zero)
    {
        this->Normal = math::Normalize(this->Normal);
    }
}

physics::Plane physics::Plane::FromPoints(const math::Vector3& A,
                                          const math::Vector3& B,
                                          const math::Vector3& C)
{
    physics::Plane Result;
    Result.Normal = math::Cross(B - A, C - A);
    Result.Distance = math::Dot(Result.Normal, A);
    return Result;
}

bool physics::Plane::IsValid() const
{
    return Normal.LengthSquared() > PHYSICS_REALC(0.0);
}

bool physics::Plane::Overlaps(const physics::Shape& Other) const
{
    switch (Other.Type)
    {
        case ShapeType::Plane:
            return ::physics::Overlaps(*this, static_cast<const Plane&>(Other));
        case ShapeType::AABox:
            return ::physics::Overlaps(*this, static_cast<const AABox&>(Other));
        case ShapeType::Sphere:
            return ::physics::Overlaps(*this, static_cast<const Sphere&>(Other));
        default:
            assert(false);
    }
    return true;
}

bool physics::Overlaps(const AABox& A, const AABox& B)
{
    if (!A.IsValid() || !B.IsValid())
    {
        return false;
    }

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
    if (!A.IsValid() || !B.IsValid())
    {
        return false;
    }

    const math::Vector3 CenterDiff = A.Center - B.Center;
    const real RadiusSum = A.Radius + B.Radius;
    return CenterDiff.LengthSquared() <= RadiusSum * RadiusSum;
}

bool physics::Overlaps(const physics::Plane& A, const physics::Plane& B)
{
    if (!A.IsValid() || !B.IsValid())
    {
        return false;
    }
    constexpr real kEpsilon = PHYSICS_REALC(0.0001);
    const bool AreParallel =
        math::IsEqual(math::Abs(math::Dot(A.Normal, B.Normal)), PHYSICS_REALC(1.0), kEpsilon);

    if (AreParallel)
    {
        return A.Distance == B.Distance;
    }
    return true;
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

bool physics::Overlaps(const physics::AABox& A, const physics::Plane& B)
{
    // TODO(Marko): Implement
    PHYSICS_UNUSED(A);
    PHYSICS_UNUSED(B);
    return false;
}

bool physics::Overlaps(const physics::Plane& A, const physics::AABox& B)
{
    return Overlaps(B, A);
}

bool physics::Overlaps(const physics::Sphere& A, const physics::Plane& B)
{
    // TODO(Marko): Implement
    PHYSICS_UNUSED(A);
    PHYSICS_UNUSED(B);
    return false;
}

bool physics::Overlaps(const physics::Plane& A, const physics::Sphere& B)
{
    return Overlaps(B, A);
}
