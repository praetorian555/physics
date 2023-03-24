#include "physics/shapes-helpers.h"

#include "physics/shapes.h"

math::Point3 Physics::ClosestPoint(const math::Point3& point, const Physics::Plane& plane)
{
    PHYSICS_ASSERT(plane.IsValid());
    const real distance = math::Dot(point - math::Point3::Zero, plane.normal) - plane.distance;
    return point - plane.normal * distance;
}

math::Point3 Physics::ClosestPoint(const math::Point3& point, const Physics::AABox& box)
{
    PHYSICS_ASSERT(box.IsValid());
    math::Point3 result = point;
    result.X = math::Clamp(result.X, box.min.X, box.max.X);
    result.Y = math::Clamp(result.Y, box.min.Y, box.max.Y);
    result.Z = math::Clamp(result.Z, box.min.Z, box.max.Z);
    return result;
}

math::Point3 Physics::ClosestPoint(const math::Point3& point, const Physics::Sphere& sphere)
{
    PHYSICS_ASSERT(sphere.IsValid());
    math::Vector3 direction = point - sphere.center;
    const real distance = direction.Length();
    if (distance >= sphere.radius)
    {
        direction = math::Normalize(direction);
        return sphere.center + direction * sphere.radius;
    }
    return point;
}

math::Point3 Physics::ClosestPoint(const math::Point3& point, const Physics::Box& box)
{
    PHYSICS_ASSERT(box.IsValid());
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

Physics::real Physics::Distance(const math::Point3& point, const Physics::Plane& plane)
{
    PHYSICS_ASSERT(plane.IsValid());
    return math::Dot(point - math::Point3::Zero, plane.normal) - plane.distance;
}

Physics::real Physics::Distance(const math::Point3& point, const Physics::AABox& box)
{
    PHYSICS_ASSERT(box.IsValid());
    return math::Sqrt(SquareDistance(point, box));
}

Physics::real Physics::Distance(const math::Point3& point, const Physics::Sphere& sphere)
{
    PHYSICS_ASSERT(sphere.IsValid());
    const math::Vector3 direction = point - sphere.center;
    const real distance = direction.Length();
    return distance >= sphere.radius ? distance - sphere.radius : PHYSICS_REALC(0.0);
}

Physics::real Physics::Distance(const math::Point3& point, const Physics::Box& box)
{
    PHYSICS_ASSERT(box.IsValid());
    const math::Point3 closest_point = ClosestPoint(point, box);
    return math::Distance(point, closest_point);
}

Physics::real Physics::SquareDistance(const math::Point3& point, const Physics::AABox& box)
{
    PHYSICS_ASSERT(box.IsValid());
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

void Physics::Enclose(Physics::AABox& out_box,
                      const Physics::AABox& in_box0,
                      const Physics::AABox& in_box1)
{
    PHYSICS_ASSERT(in_box0.IsValid());
    PHYSICS_ASSERT(in_box1.IsValid());
    out_box.min = math::Min(in_box0.min, in_box1.min);
    out_box.max = math::Max(in_box0.max, in_box1.max);
}

void Physics::Enclose(Physics::Sphere& out_sphere,
                      const Physics::Sphere& in_sphere0,
                      const Physics::Sphere& in_sphere1)
{
    const math::Vector3 direction = in_sphere1.center - in_sphere0.center;
    const real distance2 = direction.LengthSquared();
    const real radius_diff = in_sphere1.radius - in_sphere0.radius;
    if (radius_diff * radius_diff >= distance2)
    {
        // One sphere is in the another, so enclosing sphere is the larger one of the two.
        out_sphere = in_sphere1.radius >= in_sphere0.radius ? in_sphere1 : in_sphere0;
        return;
    }
    // Spheres not overlapping of partially overlapping.
    const real distance = math::Sqrt(distance2);
    const real new_radius = (in_sphere0.radius + in_sphere1.radius + distance) * PHYSICS_REALC(0.5);
    // New center is on the line between the two sphere centers, at the distance of new_radius -
    // in_sphere0.radius. This is optimized expression that does this.
    out_sphere.radius = new_radius;
    constexpr real k_epsilon = PHYSICS_REALC(1e-6);
    out_sphere.center = in_sphere0.center;
    if (distance > k_epsilon)
    {
        out_sphere.center += direction * ((new_radius - in_sphere0.radius) / distance);
    }
}
