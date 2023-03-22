#include "physics/shapes.h"

#include <cassert>

#include "math/matrix4x4.h"

physics::AABox::AABox(const math::Point3& Min, const math::Point3& Max)
    : Shape(ShapeType::AABox), Min(Min), Max(Max)
{
}

physics::AABox::AABox() : Shape(ShapeType::AABox) {}

bool physics::AABox::IsValid() const
{
    return !Min.HasNaNs() && !Max.HasNaNs() && math::Min(Min, Max) == Min &&
           math::Max(Min, Max) == Max;
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
        case ShapeType::Box:
            return ::physics::Overlaps(*this, static_cast<const Box&>(Other));
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

physics::Sphere::Sphere(const math::Point3& Center, physics::real Radius)
    : Shape(ShapeType::Sphere), Center(Center), Radius(Radius)
{
}

bool physics::Sphere::IsValid() const
{
    return !Center.HasNaNs() && !math::IsNaN(Radius) && Radius >= PHYSICS_REALC(0.0);
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
        case ShapeType::Box:
            return ::physics::Overlaps(*this, static_cast<const Box&>(Other));
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

physics::Plane physics::Plane::FromPoints(const math::Point3& A,
                                          const math::Point3& B,
                                          const math::Point3& C)
{
    physics::Plane Result;
    Result.Normal = math::Cross(B - A, C - A);
    Result.Distance = math::Dot(Result.Normal, A - math::Point3::Zero);
    return Result;
}

bool physics::Plane::IsValid() const
{
    return !Normal.HasNaNs() && !math::IsNaN(Distance) &&
           Normal.LengthSquared() > PHYSICS_REALC(0.0);
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
        case ShapeType::Box:
            return ::physics::Overlaps(*this, static_cast<const Box&>(Other));
        default:
            assert(false);
    }
    return true;
}

physics::Box::Box()
    : Shape(ShapeType::Box),
      AxisX(math::Vector3::Zero),
      AxisY(math::Vector3::Zero),
      AxisZ(math::Vector3::Zero)
{
}

physics::Box::Box(const math::Point3& Center,
                  const math::Vector3& Extents,
                  const math::Vector3& AxisX,
                  const math::Vector3& AxisY,
                  const math::Vector3& AxisZ)
    : Shape(ShapeType::Box),
      Center(Center),
      Extents(Extents),
      AxisX(AxisX),
      AxisY(AxisY),
      AxisZ(AxisZ)
{
    if (IsValid())
    {
        this->AxisX = math::Normalize(this->AxisX);
        this->AxisY = math::Normalize(this->AxisY);
        this->AxisZ = math::Normalize(this->AxisZ);
    }
}

physics::Box::Box(const math::Point3& Center,
                  const math::Vector3& Extents,
                  const math::Matrix4x4& RotMat)
    : Shape(ShapeType::Box), Center(Center), Extents(Extents)
{
    Axes[0] = math::Vector3(RotMat.Data[0][0], RotMat.Data[1][0], RotMat.Data[2][0]);
    Axes[1] = math::Vector3(RotMat.Data[0][1], RotMat.Data[1][1], RotMat.Data[2][1]);
    Axes[2] = math::Vector3(RotMat.Data[0][2], RotMat.Data[1][2], RotMat.Data[2][2]);
    if (IsValid())
    {
        AxisX = math::Normalize(Axes[0]);
        AxisY = math::Normalize(Axes[1]);
        AxisZ = math::Normalize(Axes[2]);
    }
}

bool physics::Box::IsValid() const
{
    constexpr real kEpsilon = PHYSICS_REALC(1e-6);
    return !Center.HasNaNs() && !Extents.HasNaNs() && !AxisX.HasNaNs() && !AxisY.HasNaNs() &&
           !AxisZ.HasNaNs() && Extents.X >= PHYSICS_REALC(0.0) && Extents.Y >= PHYSICS_REALC(0.0) &&
           Extents.Z >= PHYSICS_REALC(0.0) && AxisX.LengthSquared() > kEpsilon &&
           AxisY.LengthSquared() > kEpsilon && AxisZ.LengthSquared() > kEpsilon;
}

bool physics::Box::Overlaps(const physics::Shape& Other) const
{
    switch (Other.Type)
    {
        case ShapeType::AABox:
            return ::physics::Overlaps(*this, static_cast<const AABox&>(Other));
        case ShapeType::Sphere:
            return ::physics::Overlaps(*this, static_cast<const Sphere&>(Other));
        case ShapeType::Plane:
            return ::physics::Overlaps(*this, static_cast<const Plane&>(Other));
        case ShapeType::Box:
            return ::physics::Overlaps(*this, static_cast<const Box&>(Other));
        default:
            assert(false);
    }
    return false;
}

physics::real physics::Box::GetSurfaceArea() const
{
    return 2 * Extents.X * Extents.Y + 2 * Extents.X * Extents.Z + 2 * Extents.Y * Extents.Z;
}

physics::real physics::Box::GetVolume() const
{
    return Extents.X * Extents.Y * Extents.Z;
}

bool physics::Overlaps(const AABox& A, const AABox& B)
{
    assert(A.IsValid());
    assert(B.IsValid());

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
    assert(A.IsValid());
    assert(B.IsValid());

    const math::Vector3 CenterDiff = A.Center - B.Center;
    const real RadiusSum = A.Radius + B.Radius;
    return CenterDiff.LengthSquared() <= RadiusSum * RadiusSum;
}

bool physics::Overlaps(const physics::Plane& A, const physics::Plane& B)
{
    assert(A.IsValid());
    assert(B.IsValid());
    constexpr real kEpsilon = PHYSICS_REALC(0.0001);
    const bool AreParallel =
        math::IsEqual(math::Abs(math::Dot(A.Normal, B.Normal)), PHYSICS_REALC(1.0), kEpsilon);

    if (AreParallel)
    {
        return math::IsEqual(A.Distance, B.Distance, kEpsilon);
    }
    return true;
}

bool physics::Overlaps(const physics::Box& A, const physics::Box& B)
{
    assert(A.IsValid());
    assert(B.IsValid());

    math::Matrix4x4 R;
    math::Matrix4x4 AbsR;

    // Compute the rotation matrix that represents B's orientation in A's coordinate frame.
    // This is equivalent to R = A_transpose * B.
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; j++)
        {
            R.Data[i][j] = math::Dot(A.Axes[i], B.Axes[j]);
        }
    }

    // Convert distance between centers to A's coordinate frame.
    math::Vector3 Distance = B.Center - A.Center;
    Distance = math::Vector3(math::Dot(Distance, A.Axes[0]), math::Dot(Distance, A.Axes[1]),
                             math::Dot(Distance, A.Axes[2]));

    // Since we are projecting the extents, we don't really care about the sign of the rotation
    // matrix. We can just take the absolute value of each component.
    constexpr real kEpsilon = PHYSICS_REALC(0.0001);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            AbsR.Data[i][j] = math::Abs(R.Data[i][j]) + kEpsilon;
        }
    }

    // Separating-axis test for axes of A.
    for (int i = 0; i < 3; ++i)
    {
        const real ProjectionA = A.Extents[i];
        const real ProjectionB = B.Extents.X * AbsR.Data[i][0] + B.Extents.Y * AbsR.Data[i][1] +
                                 B.Extents.Z * AbsR.Data[i][2];
        if (math::Abs(Distance[i]) > ProjectionA + ProjectionB)
        {
            return false;
        }
    }

    // Separating-axis test for axes of B. We use a transpose of R effectively to get extents of A
    // in B's coordinate frame.
    for (int i = 0; i < 3; ++i)
    {
        const real ProjectionA = A.Extents.X * AbsR.Data[0][i] + A.Extents.Y * AbsR.Data[1][i] +
                                 A.Extents.Z * AbsR.Data[2][i];
        const real ProjectionB = B.Extents[i];
        const real DistanceProjection = math::Abs(
            Distance.X * R.Data[0][i] + Distance.Y * R.Data[1][i] + Distance.Z * R.Data[2][i]);
        if (DistanceProjection > ProjectionA + ProjectionB)
        {
            return false;
        }
    }

    // Test axis A0 x B0
    {
        const real ProjectionA = A.Extents.Y * AbsR.Data[2][0] + A.Extents.Z * AbsR.Data[1][0];
        const real ProjectionB = B.Extents.Y * AbsR.Data[0][2] + B.Extents.Z * AbsR.Data[0][1];
        const real DistanceProjection =
            math::Abs(Distance.Z * R.Data[1][0] - Distance.Y * R.Data[2][0]);
        if (DistanceProjection > ProjectionA + ProjectionB)
        {
            return false;
        }
    }

    // Test axis A0 x B1
    {
        const real ProjectionA = A.Extents.Y * AbsR.Data[2][1] + A.Extents.Z * AbsR.Data[1][1];
        const real ProjectionB = B.Extents.X * AbsR.Data[0][2] + B.Extents.Z * AbsR.Data[0][0];
        const real DistanceProjection =
            math::Abs(Distance.Z * R.Data[1][1] - Distance.Y * R.Data[2][1]);
        if (DistanceProjection > ProjectionA + ProjectionB)
        {
            return false;
        }
    }

    // Test axis A0 x B2
    {
        const real ProjectionA = A.Extents.Y * AbsR.Data[2][2] + A.Extents.Z * AbsR.Data[1][2];
        const real ProjectionB = B.Extents.X * AbsR.Data[0][1] + B.Extents.Y * AbsR.Data[0][0];
        const real DistanceProjection =
            math::Abs(Distance.Z * R.Data[1][2] - Distance.Y * R.Data[2][2]);
        if (DistanceProjection > ProjectionA + ProjectionB)
        {
            return false;
        }
    }

    // Test axis A1 x B0
    {
        const real ProjectionA = A.Extents.X * AbsR.Data[2][0] + A.Extents.Z * AbsR.Data[0][0];
        const real ProjectionB = B.Extents.Y * AbsR.Data[1][2] + B.Extents.Z * AbsR.Data[1][1];
        const real DistanceProjection =
            math::Abs(Distance.X * R.Data[2][0] - Distance.Z * R.Data[0][0]);
        if (DistanceProjection > ProjectionA + ProjectionB)
        {
            return false;
        }
    }

    // Test axis A1 x B1
    {
        const real ProjectionA = A.Extents.X * AbsR.Data[2][1] + A.Extents.Z * AbsR.Data[0][1];
        const real ProjectionB = B.Extents.X * AbsR.Data[1][2] + B.Extents.Z * AbsR.Data[1][0];
        const real DistanceProjection =
            math::Abs(Distance.X * R.Data[2][1] - Distance.Z * R.Data[0][1]);
        if (DistanceProjection > ProjectionA + ProjectionB)
        {
            return false;
        }
    }

    // Test axis A1 x B2
    {
        const real ProjectionA = A.Extents.X * AbsR.Data[2][2] + A.Extents.Z * AbsR.Data[0][2];
        const real ProjectionB = B.Extents.X * AbsR.Data[1][1] + B.Extents.Y * AbsR.Data[1][0];
        const real DistanceProjection =
            math::Abs(Distance.X * R.Data[2][2] - Distance.Z * R.Data[0][2]);
        if (DistanceProjection > ProjectionA + ProjectionB)
        {
            return false;
        }
    }

    // Test axis A2 x B0
    {
        const real ProjectionA = A.Extents.X * AbsR.Data[1][0] + A.Extents.Y * AbsR.Data[0][0];
        const real ProjectionB = B.Extents.Y * AbsR.Data[2][2] + B.Extents.Z * AbsR.Data[2][1];
        const real DistanceProjection =
            math::Abs(Distance.Y * R.Data[0][0] - Distance.X * R.Data[1][0]);
        if (DistanceProjection > ProjectionA + ProjectionB)
        {
            return false;
        }
    }

    // Test axis A2 x B1
    {
        const real ProjectionA = A.Extents.X * AbsR.Data[1][1] + A.Extents.Y * AbsR.Data[0][1];
        const real ProjectionB = B.Extents.X * AbsR.Data[2][2] + B.Extents.Z * AbsR.Data[2][0];
        const real DistanceProjection =
            math::Abs(Distance.Y * R.Data[0][1] - Distance.X * R.Data[1][1]);
        if (DistanceProjection > ProjectionA + ProjectionB)
        {
            return false;
        }
    }

    // Test axis A2 x B2
    {
        const real ProjectionA = A.Extents.X * AbsR.Data[1][2] + A.Extents.Y * AbsR.Data[0][2];
        const real ProjectionB = B.Extents.X * AbsR.Data[2][1] + B.Extents.Y * AbsR.Data[2][0];
        const real DistanceProjection =
            math::Abs(Distance.Y * R.Data[0][2] - Distance.X * R.Data[1][2]);
        if (DistanceProjection > ProjectionA + ProjectionB)
        {
            return false;
        }
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
    assert(A.IsValid());
    assert(B.IsValid());

    const math::Point3& Center = (A.Max + A.Min) * PHYSICS_REALC(0.5);
    const math::Vector3& Extents = A.Max - Center;

    // We need to project the extents onto the normal of the plane and find the largest possible
    // projection. This is why we need to take the absolute value of each component of the normal.
    const real Radius = Extents.X * math::Abs(B.Normal.X) + Extents.Y * math::Abs(B.Normal.Y) +
                        Extents.Z * math::Abs(B.Normal.Z);
    const real Distance = math::Dot(Center - math::Point3::Zero, B.Normal) - B.Distance;

    return math::Abs(Distance) <= Radius;
}

bool physics::Overlaps(const physics::Plane& A, const physics::AABox& B)
{
    return Overlaps(B, A);
}

bool physics::Overlaps(const physics::Sphere& A, const physics::Plane& B)
{
    assert(A.IsValid());
    assert(B.IsValid());
    const real Distance = math::Dot(A.Center - math::Point3::Zero, B.Normal) - B.Distance;
    return math::Abs(Distance) <= A.Radius;
}

bool physics::Overlaps(const physics::Plane& A, const physics::Sphere& B)
{
    return Overlaps(B, A);
}

bool physics::Overlaps(const physics::Box& A, const physics::AABox& B)
{
    assert(A.IsValid());
    assert(B.IsValid());
    const math::Point3 AABBCenter = (B.Min + B.Max) * PHYSICS_REALC(0.5);
    const math::Vector3 AABBExtents = B.Max - AABBCenter;
    const physics::Box BB(AABBCenter, AABBExtents, math::Matrix4x4{});
    return Overlaps(A, BB);
}

bool physics::Overlaps(const physics::AABox& A, const physics::Box& B)
{
    return Overlaps(B, A);
}

bool physics::Overlaps(const physics::Box& A, const physics::Sphere& B)
{
    assert(A.IsValid());
    assert(B.IsValid());

    const math::Point3 Point = ClosestPoint(B.Center, A);
    const math::Vector3 DistanceVector = Point - B.Center;
    return math::Dot(DistanceVector, DistanceVector) <= B.Radius * B.Radius;
}

bool physics::Overlaps(const physics::Sphere& A, const physics::Box& B)
{
    return Overlaps(B, A);
}

bool physics::Overlaps(const physics::Box& A, const physics::Plane& B)
{
    assert(A.IsValid());
    assert(B.IsValid());

    // Project the farthest point of the box onto the plane normal.
    const real Projection = A.Extents.X * math::Dot(A.Axes[0], B.Normal) +
                            A.Extents.Y * math::Dot(A.Axes[1], B.Normal) +
                            A.Extents.Z * math::Dot(A.Axes[2], B.Normal);
    const real Distance = math::Dot(A.Center - math::Point3::Zero, B.Normal) - B.Distance;
    return math::Abs(Distance) <= Projection;
}

bool physics::Overlaps(const physics::Plane& A, const physics::Box& B)
{
    return Overlaps(B, A);
}

math::Point3 physics::ClosestPoint(const math::Point3& Point, const physics::Plane& Plane)
{
    assert(Plane.IsValid());
    assert(Plane.IsValid());
    const real Distance = math::Dot(Point - math::Point3::Zero, Plane.Normal) - Plane.Distance;
    return Point - Plane.Normal * Distance;
}

math::Point3 physics::ClosestPoint(const math::Point3& Point, const physics::AABox& Box)
{
    assert(Box.IsValid());
    math::Point3 Result = Point;
    Result.X = math::Clamp(Result.X, Box.Min.X, Box.Max.X);
    Result.Y = math::Clamp(Result.Y, Box.Min.Y, Box.Max.Y);
    Result.Z = math::Clamp(Result.Z, Box.Min.Z, Box.Max.Z);
    return Result;
}

math::Point3 physics::ClosestPoint(const math::Point3& Point, const physics::Sphere& Sphere)
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

math::Point3 physics::ClosestPoint(const math::Point3& Point, const physics::Box& Box)
{
    assert(Box.IsValid());
    math::Point3 Result = Box.Center;
    const math::Vector3 Direction = Point - Box.Center;
    for (int AxisIdx = 0; AxisIdx < 3; ++AxisIdx)
    {
        real Distance = math::Dot(Direction, Box.Axes[AxisIdx]);
        Distance = math::Clamp(Distance, -Box.Extents[AxisIdx], Box.Extents[AxisIdx]);
        Result += Distance * Box.Axes[AxisIdx];
    }
    return Result;
}

physics::real physics::Distance(const math::Point3& Point, const physics::Plane& Plane)
{
    assert(Plane.IsValid());
    return math::Dot(Point - math::Point3::Zero, Plane.Normal) - Plane.Distance;
}

physics::real physics::Distance(const math::Point3& Point, const physics::AABox& Box)
{
    assert(Box.IsValid());
    return math::Sqrt(SquareDistance(Point, Box));
}

physics::real physics::Distance(const math::Point3& Point, const physics::Sphere& Sphere)
{
    assert(Sphere.IsValid());
    const math::Vector3 Direction = Point - Sphere.Center;
    const real Distance = Direction.Length();
    return Distance >= Sphere.Radius ? Distance - Sphere.Radius : PHYSICS_REALC(0.0);
}

physics::real physics::Distance(const math::Point3& Point, const physics::Box& Box)
{
    assert(Box.IsValid());
    const math::Point3 ClosestPointOnBox = ClosestPoint(Point, Box);
    return math::Distance(Point, ClosestPointOnBox);
}

physics::real physics::SquareDistance(const math::Point3& Point, const physics::AABox& Box)
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
