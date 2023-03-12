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
    const real SquareDistance = physics::SquareDistance(B.Center, A);
    return SquareDistance <= B.Radius * B.Radius;
}

bool physics::Overlaps(const Sphere& A, const AABox& B)
{
    return Overlaps(B, A);
}

bool physics::Overlaps(const physics::AABox& A, const physics::Plane& B)
{
    if (!A.IsValid() || !B.IsValid())
    {
        return false;
    }

    const math::Vector3& Center = (A.Max + A.Min) * PHYSICS_REALC(0.5);
    const math::Vector3& Extents = A.Max - Center;

    // We need to project the extents onto the normal of the plane and find the largest possible
    // projection. This is why we need to take the absolute value of each component of the normal.
    const real Radius = Extents.X * math::Abs(B.Normal.X) + Extents.Y * math::Abs(B.Normal.Y) +
                        Extents.Z * math::Abs(B.Normal.Z);
    const real Distance = math::Dot(Center, B.Normal) - B.Distance;

    return math::Abs(Distance) <= Radius;
}

bool physics::Overlaps(const physics::Plane& A, const physics::AABox& B)
{
    return Overlaps(B, A);
}

bool physics::Overlaps(const physics::Sphere& A, const physics::Plane& B)
{
    if (!A.IsValid() || !B.IsValid())
    {
        return false;
    }
    const real Distance = math::Dot(A.Center, B.Normal) - B.Distance;
    return math::Abs(Distance) <= A.Radius;
}

bool physics::Overlaps(const physics::Plane& A, const physics::Sphere& B)
{
    return Overlaps(B, A);
}

math::Vector3 physics::ClosestPoint(const math::Vector3& Point, const physics::Plane& Plane)
{
    assert(Plane.IsValid());
    const real Distance = math::Dot(Point, Plane.Normal) - Plane.Distance;
    return Point - Plane.Normal * Distance;
}

math::Vector3 physics::ClosestPoint(const math::Vector3& Point, const physics::AABox& Box)
{
    assert(Box.IsValid());
    math::Vector3 Result = Point;
    Result.X = math::Clamp(Result.X, Box.Min.X, Box.Max.X);
    Result.Y = math::Clamp(Result.Y, Box.Min.Y, Box.Max.Y);
    Result.Z = math::Clamp(Result.Z, Box.Min.Z, Box.Max.Z);
    return Result;
}

math::Vector3 physics::ClosestPoint(const math::Vector3& Point, const physics::Sphere& Sphere)
{
    assert(Sphere.IsValid());
    math::Vector3 Direction = Point - Sphere.Center;
    const real Distance = Direction.Length();
    if (Distance >= Sphere.Radius)
    {
        Direction = math::Normalize(Direction);
        return Sphere.Center + Direction * Sphere.Radius;
    }
    return Point;
}

physics::real physics::Distance(const math::Vector3& Point, const physics::Plane& Plane)
{
    assert(Plane.IsValid());
    return math::Dot(Point, Plane.Normal) - Plane.Distance;
}

physics::real physics::Distance(const math::Vector3& Point, const physics::AABox& Box)
{
    return math::Sqrt(SquareDistance(Point, Box));
}

physics::real physics::Distance(const math::Vector3& Point, const physics::Sphere& Sphere)
{
    assert(Sphere.IsValid());
    const math::Vector3 Direction = Point - Sphere.Center;
    const real Distance = Direction.Length();
    return Distance >= Sphere.Radius ? Distance - Sphere.Radius : PHYSICS_REALC(0.0);
}

physics::real physics::SquareDistance(const math::Vector3& Point, const physics::AABox& Box)
{
    assert(Box.IsValid());
    real SquareDistance = PHYSICS_REALC(0.0);
    for (int AxisIndex = 0; AxisIndex < 3; AxisIndex++)
    {
        const real AxisValue = Point[AxisIndex];
        if (AxisValue < Box.Min[AxisIndex])
        {
            const real Diff = Box.Min[AxisIndex] - AxisValue;
            SquareDistance += Diff * Diff;
        }
        else if (AxisValue > Box.Max[AxisIndex])
        {
            const real Diff = AxisValue - Box.Max[AxisIndex];
            SquareDistance += Diff * Diff;
        }
    }
    return SquareDistance;
}
