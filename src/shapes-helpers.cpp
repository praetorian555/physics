#include "physics/shapes-helpers.h"

#include "physics/shapes.h"

math::Point3 physics::ClosestPoint(const math::Point3& point, const physics::Plane& plane)
{
    assert(plane.IsValid());
    const real distance = math::Dot(point - math::Point3::Zero, plane.normal) - plane.distance;
    return point - plane.normal * distance;
}

math::Point3 physics::ClosestPoint(const math::Point3& point, const physics::AABox& box)
{
    assert(box.IsValid());
    math::Point3 result = point;
    result.X = math::Clamp(result.X, box.min.X, box.max.X);
    result.Y = math::Clamp(result.Y, box.min.Y, box.max.Y);
    result.Z = math::Clamp(result.Z, box.min.Z, box.max.Z);
    return result;
}

math::Point3 physics::ClosestPoint(const math::Point3& point, const physics::Sphere& sphere)
{
    assert(sphere.IsValid());
    math::Vector3 direction = point - sphere.center;
    const real distance = direction.Length();
    if (distance >= sphere.radius)
    {
        direction = math::Normalize(direction);
        return sphere.center + direction * sphere.radius;
    }
    return point;
}

math::Point3 physics::ClosestPoint(const math::Point3& point, const physics::Box& box)
{
    assert(box.IsValid());
    math::Point3 result = box.center;
    const math::Vector3 direction = point - box.center;
    for (int i = 0; i < 3; ++i)
    {
        real distance = math::Dot(direction, box.axes[i]);
        distance = math::Clamp(distance, -box.half_extents[i], box.half_extents[i]);
        result += distance * box.axes[i];
    }
    return result;
}

physics::real physics::Distance(const math::Point3& point, const physics::Plane& plane)
{
    assert(plane.IsValid());
    return math::Dot(point - math::Point3::Zero, plane.normal) - plane.distance;
}

physics::real physics::Distance(const math::Point3& point, const physics::AABox& box)
{
    assert(box.IsValid());
    return math::Sqrt(SquareDistance(point, box));
}

physics::real physics::Distance(const math::Point3& point, const physics::Sphere& sphere)
{
    assert(sphere.IsValid());
    const math::Vector3 direction = point - sphere.center;
    const real distance = direction.Length();
    return distance >= sphere.radius ? distance - sphere.radius : PHYSICS_REALC(0.0);
}

physics::real physics::Distance(const math::Point3& point, const physics::Box& box)
{
    assert(box.IsValid());
    const math::Point3 closest_point = ClosestPoint(point, box);
    return math::Distance(point, closest_point);
}

physics::real physics::SquareDistance(const math::Point3& point, const physics::AABox& box)
{
    assert(box.IsValid());
    real square_distance = PHYSICS_REALC(0.0);
    for (int i = 0; i < 3; i++)
    {
        const real axis_value = point[i];
        if (axis_value < box.min[i])
        {
            const real diff = box.min[i] - axis_value;
            square_distance += diff * diff;
        }
        else if (axis_value > box.max[i])
        {
            const real diff = axis_value - box.max[i];
            square_distance += diff * diff;
        }
    }
    return square_distance;
}
