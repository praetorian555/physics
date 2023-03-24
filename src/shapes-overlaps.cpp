#include "physics/shapes-overlaps.h"

#include "math/matrix4x4.h"

#include "physics/shapes-helpers.h"
#include "physics/shapes.h"

bool Physics::Overlaps(const AABox& a, const AABox& b)
{
    PHYSICS_ASSERT(a.IsValid());
    PHYSICS_ASSERT(b.IsValid());

    if (a.max.X < b.min.X || a.min.X > b.max.X)
    {
        return false;
    }
    if (a.max.Y < b.min.Y || a.min.Y > b.max.Y)
    {
        return false;
    }
    if (a.max.Z < b.min.Z || a.min.Z > b.max.Z)
    {
        return false;
    }
    return true;
}

bool Physics::Overlaps(const Sphere& a, const Sphere& b)
{
    PHYSICS_ASSERT(a.IsValid());
    PHYSICS_ASSERT(b.IsValid());

    const math::Vector3 center_diff = a.center - b.center;
    const real radius_sum = a.radius + b.radius;
    return center_diff.LengthSquared() <= radius_sum * radius_sum;
}

bool Physics::Overlaps(const Physics::Plane& a, const Physics::Plane& b)
{
    PHYSICS_ASSERT(a.IsValid());
    PHYSICS_ASSERT(b.IsValid());
    constexpr real k_epsilon = PHYSICS_REALC(0.0001);
    const real abs_projection = math::Abs(math::Dot(a.normal, b.normal));
    const bool are_parallel = math::IsEqual(abs_projection, PHYSICS_REALC(1.0), k_epsilon);
    if (are_parallel)
    {
        return math::IsEqual(a.distance, b.distance, k_epsilon);
    }
    return true;
}

bool Physics::Overlaps(const Physics::Box& a, const Physics::Box& b)
{
    PHYSICS_ASSERT(a.IsValid());
    PHYSICS_ASSERT(b.IsValid());

    math::Matrix4x4 rot_matrix;
    math::Matrix4x4 abs_rot_matrix;

    // Compute the rotation matrix that represents b's orientation in a's coordinate frame.
    // This is equivalent to rot_matrix = A_transpose * b.
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; j++)
        {
            rot_matrix.Data[i][j] = math::Dot(a.axes[i], b.axes[j]);
        }
    }

    // Convert distance between centers to a's coordinate frame.
    math::Vector3 distance = b.center - a.center;
    distance = math::Vector3(math::Dot(distance, a.axes[0]), math::Dot(distance, a.axes[1]),
                             math::Dot(distance, a.axes[2]));

    // Since we are projecting the extents, we don't really care about the sign of the rotation
    // matrix. We can just take the absolute value of each component.
    constexpr real k_epsilon = PHYSICS_REALC(0.0001);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            abs_rot_matrix.Data[i][j] = math::Abs(rot_matrix.Data[i][j]) + k_epsilon;
        }
    }

    // Separating-axis test for axes of a.
    for (int i = 0; i < 3; ++i)
    {
        const real projection_a = a.half_extents[i];
        const real projection_b = b.half_extents.X * abs_rot_matrix.Data[i][0] +
                                  b.half_extents.Y * abs_rot_matrix.Data[i][1] +
                                  b.half_extents.Z * abs_rot_matrix.Data[i][2];
        if (math::Abs(distance[i]) > projection_a + projection_b)
        {
            return false;
        }
    }

    // Separating-axis test for axes of b. We use a transpose of rot_matrix effectively to get
    // extents of a in b's coordinate frame.
    for (int i = 0; i < 3; ++i)
    {
        const real projection_a = a.half_extents.X * abs_rot_matrix.Data[0][i] +
                                  a.half_extents.Y * abs_rot_matrix.Data[1][i] +
                                  a.half_extents.Z * abs_rot_matrix.Data[2][i];
        const real projection_b = b.half_extents[i];
        const real distance_projection =
            math::Abs(distance.X * rot_matrix.Data[0][i] + distance.Y * rot_matrix.Data[1][i] +
                      distance.Z * rot_matrix.Data[2][i]);
        if (distance_projection > projection_a + projection_b)
        {
            return false;
        }
    }

    // Test axis A0 x B0
    {
        const real projection_a = a.half_extents.Y * abs_rot_matrix.Data[2][0] +
                                  a.half_extents.Z * abs_rot_matrix.Data[1][0];
        const real projection_b = b.half_extents.Y * abs_rot_matrix.Data[0][2] +
                                  b.half_extents.Z * abs_rot_matrix.Data[0][1];
        const real distance_projection =
            math::Abs(distance.Z * rot_matrix.Data[1][0] - distance.Y * rot_matrix.Data[2][0]);
        if (distance_projection > projection_a + projection_b)
        {
            return false;
        }
    }

    // Test axis A0 x B1
    {
        const real projection_a = a.half_extents.Y * abs_rot_matrix.Data[2][1] +
                                  a.half_extents.Z * abs_rot_matrix.Data[1][1];
        const real projection_b = b.half_extents.X * abs_rot_matrix.Data[0][2] +
                                  b.half_extents.Z * abs_rot_matrix.Data[0][0];
        const real distance_projection =
            math::Abs(distance.Z * rot_matrix.Data[1][1] - distance.Y * rot_matrix.Data[2][1]);
        if (distance_projection > projection_a + projection_b)
        {
            return false;
        }
    }

    // Test axis A0 x B2
    {
        const real projection_a = a.half_extents.Y * abs_rot_matrix.Data[2][2] +
                                  a.half_extents.Z * abs_rot_matrix.Data[1][2];
        const real projection_b = b.half_extents.X * abs_rot_matrix.Data[0][1] +
                                  b.half_extents.Y * abs_rot_matrix.Data[0][0];
        const real distance_projection =
            math::Abs(distance.Z * rot_matrix.Data[1][2] - distance.Y * rot_matrix.Data[2][2]);
        if (distance_projection > projection_a + projection_b)
        {
            return false;
        }
    }

    // Test axis A1 x B0
    {
        const real projection_a = a.half_extents.X * abs_rot_matrix.Data[2][0] +
                                  a.half_extents.Z * abs_rot_matrix.Data[0][0];
        const real projection_b = b.half_extents.Y * abs_rot_matrix.Data[1][2] +
                                  b.half_extents.Z * abs_rot_matrix.Data[1][1];
        const real distance_projection =
            math::Abs(distance.X * rot_matrix.Data[2][0] - distance.Z * rot_matrix.Data[0][0]);
        if (distance_projection > projection_a + projection_b)
        {
            return false;
        }
    }

    // Test axis A1 x B1
    {
        const real projection_a = a.half_extents.X * abs_rot_matrix.Data[2][1] +
                                  a.half_extents.Z * abs_rot_matrix.Data[0][1];
        const real projection_b = b.half_extents.X * abs_rot_matrix.Data[1][2] +
                                  b.half_extents.Z * abs_rot_matrix.Data[1][0];
        const real distance_projection =
            math::Abs(distance.X * rot_matrix.Data[2][1] - distance.Z * rot_matrix.Data[0][1]);
        if (distance_projection > projection_a + projection_b)
        {
            return false;
        }
    }

    // Test axis A1 x B2
    {
        const real projection_a = a.half_extents.X * abs_rot_matrix.Data[2][2] +
                                  a.half_extents.Z * abs_rot_matrix.Data[0][2];
        const real projection_b = b.half_extents.X * abs_rot_matrix.Data[1][1] +
                                  b.half_extents.Y * abs_rot_matrix.Data[1][0];
        const real distance_projection =
            math::Abs(distance.X * rot_matrix.Data[2][2] - distance.Z * rot_matrix.Data[0][2]);
        if (distance_projection > projection_a + projection_b)
        {
            return false;
        }
    }

    // Test axis A2 x B0
    {
        const real projection_a = a.half_extents.X * abs_rot_matrix.Data[1][0] +
                                  a.half_extents.Y * abs_rot_matrix.Data[0][0];
        const real projection_b = b.half_extents.Y * abs_rot_matrix.Data[2][2] +
                                  b.half_extents.Z * abs_rot_matrix.Data[2][1];
        const real distance_projection =
            math::Abs(distance.Y * rot_matrix.Data[0][0] - distance.X * rot_matrix.Data[1][0]);
        if (distance_projection > projection_a + projection_b)
        {
            return false;
        }
    }

    // Test axis A2 x B1
    {
        const real projection_a = a.half_extents.X * abs_rot_matrix.Data[1][1] +
                                  a.half_extents.Y * abs_rot_matrix.Data[0][1];
        const real projection_b = b.half_extents.X * abs_rot_matrix.Data[2][2] +
                                  b.half_extents.Z * abs_rot_matrix.Data[2][0];
        const real distance_projection =
            math::Abs(distance.Y * rot_matrix.Data[0][1] - distance.X * rot_matrix.Data[1][1]);
        if (distance_projection > projection_a + projection_b)
        {
            return false;
        }
    }

    // Test axis A2 x B2
    {
        const real projection_a = a.half_extents.X * abs_rot_matrix.Data[1][2] +
                                  a.half_extents.Y * abs_rot_matrix.Data[0][2];
        const real projection_b = b.half_extents.X * abs_rot_matrix.Data[2][1] +
                                  b.half_extents.Y * abs_rot_matrix.Data[2][0];
        const real distance_projection =
            math::Abs(distance.Y * rot_matrix.Data[0][2] - distance.X * rot_matrix.Data[1][2]);
        if (distance_projection > projection_a + projection_b)
        {
            return false;
        }
    }

    return true;
}

bool Physics::Overlaps(const AABox& a, const Sphere& b)
{
    const real square_distance = Physics::SquareDistance(b.center, a);
    return square_distance <= b.radius * b.radius;
}

bool Physics::Overlaps(const Sphere& a, const AABox& b)
{
    return Overlaps(b, a);
}

bool Physics::Overlaps(const Physics::AABox& a, const Physics::Plane& b)
{
    PHYSICS_ASSERT(a.IsValid());
    PHYSICS_ASSERT(b.IsValid());

    const math::Point3& center = (a.max + a.min) * PHYSICS_REALC(0.5);
    const math::Vector3& half_extents = a.max - center;

    // We need to project the extents onto the normal of the plane and find the largest possible
    // projection. This is why we need to take the absolute value of each component of the normal.
    const real radius = half_extents.X * math::Abs(b.normal.X) +
                        half_extents.Y * math::Abs(b.normal.Y) +
                        half_extents.Z * math::Abs(b.normal.Z);
    const real distance = math::Dot(center - math::Point3::Zero, b.normal) - b.distance;

    return math::Abs(distance) <= radius;
}

bool Physics::Overlaps(const Physics::Plane& a, const Physics::AABox& b)
{
    return Overlaps(b, a);
}

bool Physics::Overlaps(const Physics::Sphere& a, const Physics::Plane& b)
{
    PHYSICS_ASSERT(a.IsValid());
    PHYSICS_ASSERT(b.IsValid());
    const real distance = math::Dot(a.center - math::Point3::Zero, b.normal) - b.distance;
    return math::Abs(distance) <= a.radius;
}

bool Physics::Overlaps(const Physics::Plane& a, const Physics::Sphere& b)
{
    return Overlaps(b, a);
}

bool Physics::Overlaps(const Physics::Box& a, const Physics::AABox& b)
{
    PHYSICS_ASSERT(a.IsValid());
    PHYSICS_ASSERT(b.IsValid());
    const math::Point3 aabb_center = (b.min + b.max) * PHYSICS_REALC(0.5);
    const math::Vector3 aabb_half_extents = b.max - aabb_center;
    const Physics::Box b_box(aabb_center, aabb_half_extents, math::Matrix4x4{});
    return Overlaps(a, b_box);
}

bool Physics::Overlaps(const Physics::AABox& a, const Physics::Box& b)
{
    return Overlaps(b, a);
}

bool Physics::Overlaps(const Physics::Box& a, const Physics::Sphere& b)
{
    PHYSICS_ASSERT(a.IsValid());
    PHYSICS_ASSERT(b.IsValid());

    const math::Point3 point = ClosestPoint(b.center, a);
    const math::Vector3 distance_vector = point - b.center;
    return math::Dot(distance_vector, distance_vector) <= b.radius * b.radius;
}

bool Physics::Overlaps(const Physics::Sphere& a, const Physics::Box& b)
{
    return Overlaps(b, a);
}

bool Physics::Overlaps(const Physics::Box& a, const Physics::Plane& b)
{
    PHYSICS_ASSERT(a.IsValid());
    PHYSICS_ASSERT(b.IsValid());

    // Project the farthest point of the box onto the plane normal.
    const real projection = a.half_extents.X * math::Dot(a.axes[0], b.normal) +
                            a.half_extents.Y * math::Dot(a.axes[1], b.normal) +
                            a.half_extents.Z * math::Dot(a.axes[2], b.normal);
    const real distance = math::Dot(a.center - math::Point3::Zero, b.normal) - b.distance;
    return math::Abs(distance) <= projection;
}

bool Physics::Overlaps(const Physics::Plane& a, const Physics::Box& b)
{
    return Overlaps(b, a);
}
