#include "physics/shapes.h"

#include "math/matrix4x4.h"

#include "physics/shapes-overlaps.h"

physics::AABox::AABox(const math::Point3& in_min, const math::Point3& in_max)
    : Shape(ShapeType::AABox), min(in_min), max(in_max)
{
}

physics::AABox::AABox() : Shape(ShapeType::AABox) {}

bool physics::AABox::IsValid() const
{
    return !min.HasNaNs() && !max.HasNaNs() && math::Min(min, max) == min &&
           math::Max(min, max) == max;
}

bool physics::AABox::Overlaps(const Shape& other) const
{
    switch (other.type)
    {
        case ShapeType::AABox:
            return ::physics::Overlaps(*this, static_cast<const AABox&>(other));
        case ShapeType::Sphere:
            return ::physics::Overlaps(*this, static_cast<const Sphere&>(other));
        case ShapeType::Plane:
            return ::physics::Overlaps(*this, static_cast<const Plane&>(other));
        case ShapeType::Box:
            return ::physics::Overlaps(*this, static_cast<const Box&>(other));
        default:
            assert(false);
    }
    return false;
}

physics::real physics::AABox::GetVolume() const
{
    const math::Vector3 diagonal = max - min;
    return diagonal.X * diagonal.Y * diagonal.Z;
}

physics::real physics::AABox::GetSurfaceArea() const
{
    const math::Vector3 diagonal = max - min;
    return 2 * diagonal.X * diagonal.Y + 2 * diagonal.X * diagonal.Z + 2 * diagonal.Y * diagonal.Z;
}

physics::Sphere::Sphere() : Shape(ShapeType::Sphere), radius(PHYSICS_REALC(0.0)) {}

physics::Sphere::Sphere(const math::Point3& center, physics::real radius)
    : Shape(ShapeType::Sphere), center(center), radius(radius)
{
}

bool physics::Sphere::IsValid() const
{
    return !center.HasNaNs() && !math::IsNaN(radius) && radius >= PHYSICS_REALC(0.0);
}

bool physics::Sphere::Overlaps(const physics::Shape& other) const
{
    switch (other.type)
    {
        case ShapeType::Sphere:
            return ::physics::Overlaps(*this, static_cast<const Sphere&>(other));
        case ShapeType::AABox:
            return ::physics::Overlaps(*this, static_cast<const AABox&>(other));
        case ShapeType::Plane:
            return ::physics::Overlaps(*this, static_cast<const Plane&>(other));
        case ShapeType::Box:
            return ::physics::Overlaps(*this, static_cast<const Box&>(other));
        default:
            assert(false);
    }
    return false;
}

physics::real physics::Sphere::GetSurfaceArea() const
{
    constexpr real k_constant = PHYSICS_REALC(4.0) * math::kPi;
    return k_constant * radius * radius;
}

physics::real physics::Sphere::GetVolume() const
{
    constexpr real k_constant = PHYSICS_REALC(4.0) / PHYSICS_REALC(3.0) * math::kPi;
    return k_constant * radius * radius * radius;
}

physics::Plane::Plane() : Shape(ShapeType::Plane), distance(PHYSICS_REALC(0.0)) {}

physics::Plane::Plane(const math::Vector3& in_normal, physics::real in_distance)
    : Shape(ShapeType::Plane), normal(in_normal), distance(in_distance)
{
    if (normal != math::Vector3::Zero)
    {
        normal = math::Normalize(normal);
    }
}

physics::Plane physics::Plane::FromPoints(const math::Point3& a,
                                          const math::Point3& b,
                                          const math::Point3& c)
{
    physics::Plane result;
    result.normal = math::Cross(b - a, c - a);
    result.distance = math::Dot(result.normal, a - math::Point3::Zero);
    return result;
}

bool physics::Plane::IsValid() const
{
    return !normal.HasNaNs() && !math::IsNaN(distance) &&
           normal.LengthSquared() > PHYSICS_REALC(0.0);
}

bool physics::Plane::Overlaps(const physics::Shape& other) const
{
    switch (other.type)
    {
        case ShapeType::Plane:
            return ::physics::Overlaps(*this, static_cast<const Plane&>(other));
        case ShapeType::AABox:
            return ::physics::Overlaps(*this, static_cast<const AABox&>(other));
        case ShapeType::Sphere:
            return ::physics::Overlaps(*this, static_cast<const Sphere&>(other));
        case ShapeType::Box:
            return ::physics::Overlaps(*this, static_cast<const Box&>(other));
        default:
            assert(false);
    }
    return true;
}

physics::Box::Box()
    : Shape(ShapeType::Box),
      axis_x(math::Vector3::Zero),
      axis_y(math::Vector3::Zero),
      axis_z(math::Vector3::Zero)
{
}

physics::Box::Box(const math::Point3& in_center,
                  const math::Vector3& in_half_extents,
                  const math::Vector3& in_axis_x,
                  const math::Vector3& in_axis_y,
                  const math::Vector3& in_axis_z)
    : Shape(ShapeType::Box),
      center(in_center),
      half_extents(in_half_extents),
      axis_x(in_axis_x),
      axis_y(in_axis_y),
      axis_z(in_axis_z)
{
    if (IsValid())
    {
        axis_x = math::Normalize(axis_x);
        axis_y = math::Normalize(axis_y);
        axis_z = math::Normalize(axis_z);
    }
}

physics::Box::Box(const math::Point3& in_center,
                  const math::Vector3& in_half_extents,
                  const math::Matrix4x4& in_rotation_matrix)
    : Shape(ShapeType::Box), center(in_center), half_extents(in_half_extents)
{
    axes[0] = math::Vector3(in_rotation_matrix.Data[0][0], in_rotation_matrix.Data[1][0],
                            in_rotation_matrix.Data[2][0]);
    axes[1] = math::Vector3(in_rotation_matrix.Data[0][1], in_rotation_matrix.Data[1][1],
                            in_rotation_matrix.Data[2][1]);
    axes[2] = math::Vector3(in_rotation_matrix.Data[0][2], in_rotation_matrix.Data[1][2],
                            in_rotation_matrix.Data[2][2]);
    if (IsValid())
    {
        axes[0] = math::Normalize(axes[0]);
        axes[1] = math::Normalize(axes[1]);
        axes[2] = math::Normalize(axes[2]);
    }
}

bool physics::Box::IsValid() const
{
    constexpr real k_epsilon = PHYSICS_REALC(1e-6);
    return !center.HasNaNs() && !half_extents.HasNaNs() && !axis_x.HasNaNs() && !axis_y.HasNaNs() &&
           !axis_z.HasNaNs() && half_extents.X >= PHYSICS_REALC(0.0) &&
           half_extents.Y >= PHYSICS_REALC(0.0) && half_extents.Z >= PHYSICS_REALC(0.0) &&
           axis_x.LengthSquared() > k_epsilon && axis_y.LengthSquared() > k_epsilon &&
           axis_z.LengthSquared() > k_epsilon;
}

bool physics::Box::Overlaps(const physics::Shape& other) const
{
    switch (other.type)
    {
        case ShapeType::AABox:
            return ::physics::Overlaps(*this, static_cast<const AABox&>(other));
        case ShapeType::Sphere:
            return ::physics::Overlaps(*this, static_cast<const Sphere&>(other));
        case ShapeType::Plane:
            return ::physics::Overlaps(*this, static_cast<const Plane&>(other));
        case ShapeType::Box:
            return ::physics::Overlaps(*this, static_cast<const Box&>(other));
        default:
            assert(false);
    }
    return false;
}

physics::real physics::Box::GetSurfaceArea() const
{
    return 2 * half_extents.X * half_extents.Y + 2 * half_extents.X * half_extents.Z + 2 * half_extents.Y * half_extents.Z;
}

physics::real physics::Box::GetVolume() const
{
    return half_extents.X * half_extents.Y * half_extents.Z;
}
