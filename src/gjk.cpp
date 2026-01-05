#include "physics/gjk.hpp"

#include "opal/container/in-place-array.h"

Physics::Vector2r Physics::SignedVolume1D(const Vector3r& start, const Vector3r& end)
{
    PHYSICS_ASSERT(!Opal::IsEqual(start, end, PHYSICS_CONST(0.000001)), "Line segment must not have 0 length");
    const Vector3r line = end - start;
    const Vector3r start_to_origin = Vector3r::Zero() - start;
    const Vector3r projected_vector = start + (Opal::Dot(start_to_origin, line) * line) / Opal::LengthSquared(line);

    // Find axis on which the line has the largest projection.
    i32 axis_idx = 0;
    real max_length = 0;
    for (i32 i = 0; i < 3; ++i)
    {
        const real length = end[i] - start[i];
        if (length * length > max_length * max_length)
        {
            max_length = length;
            axis_idx = i;
        }
    }

    const real a = start[axis_idx];
    const real b = end[axis_idx];
    const real o = projected_vector[axis_idx];
    if ((o > a && o < b) || (o > b && o < a))
    {
        // If points that define line segment are A and B, and projected point is P
        // then P = lambda1 * A + lambda2 * B, where lambda1 + lambda2 = 1. The closer the P is to A, larger lambda1 should be.
        Vector2r bar_coords;
        bar_coords[0] = (b - o) / max_length;
        bar_coords[1] = (o - a) / max_length;
        return bar_coords;
    }

    if ((a <= b && o <= a) || (a >= b && o >= a))
    {
        // Projected origin is on the far side of the start point of the line segment
        return {1, 0};
    }
    // Projected origin is on the far side of the end point of the line segment
    return {0, 1};
}

namespace
{

bool IsSameSign(Physics::real a, Physics::real b)
{
    return (a > 0 && b > 0) || (a < 0 && b < 0);
}

}  // namespace

Physics::Vector3r Physics::SignedVolume2D(const Vector3r& a, const Vector3r& b, const Vector3r& c)
{
    const Vector3r normal = Opal::Cross(b - a, c - a);
    PHYSICS_ASSERT(Opal::LengthSquared(normal) > PHYSICS_CONST(0.000001) * PHYSICS_CONST(0.000001),
                   "Point of the triangle must not be colinear");
    const Vector3r projected_origin = (Opal::Dot(a, normal) * normal) / Opal::LengthSquared(normal);

    // Find the axis with the greatest projected area
    // Project the triangle to the planes XY, YZ and ZX and take the one that has the greatest area
    i32 axis_idx = 0;
    real max_area = 0;
    for (i32 i = 0; i < 3; ++i)
    {
        const i32 j = (i + 1) % 3;
        const i32 k = (i + 2) % 3;

        const Vector2r a_proj(a[j], a[k]);
        const Vector2r b_proj(b[j], b[k]);
        const Vector2r c_proj(c[j], c[k]);
        const Vector2r ab = b_proj - a_proj;
        const Vector2r ac = c_proj - a_proj;
        const real area = (ab.x * ac.y) - (ab.y * ac.x);
        if (area * area > max_area * max_area)
        {
            max_area = area;
            axis_idx = i;
        }
    }

    const i32 x_proj = (axis_idx + 1) % 3;
    const i32 y_proj = (axis_idx + 2) % 3;
    Opal::InPlaceArray<Vector2r, 3> proj_points;
    proj_points[0] = Vector2r(a[x_proj], a[y_proj]);
    proj_points[1] = Vector2r(b[x_proj], b[y_proj]);
    proj_points[2] = Vector2r(c[x_proj], c[y_proj]);
    const Vector2r origin = Vector2r(projected_origin[x_proj], projected_origin[y_proj]);

    Vector3r areas = Vector3r::Zero();
    for (i32 i = 0; i < 3; ++i)
    {
        const i32 j = (i + 1) % 3;
        const i32 k = (i + 2) % 3;
        const Vector2r a_proj = origin;
        const Vector2r b_proj = proj_points[j];
        const Vector2r c_proj = proj_points[k];
        const Vector2r ab = b_proj - a_proj;
        const Vector2r ac = c_proj - a_proj;
        areas[i] = (ab.x * ac.y) - (ab.y * ac.x);
    }

    if (IsSameSign(max_area, areas[0]) && IsSameSign(max_area, areas[1]) && IsSameSign(max_area, areas[2]))
    {
        // The origin is inside the triangle, so return the barycentric coordinates
        return areas / max_area;
    }

    // If we are here, we need to project the origin onto the edges and determine the closest point.
    real distance = 1e10;
    Vector3r barycentric_coordinates(1, 0, 0);
    for (i32 i = 0; i < 3; ++i)
    {
        const i32 j = (i + 1) % 3;
        const i32 k = (i + 2) % 3;

        Opal::InPlaceArray<Vector3r, 3> edge_points;
        edge_points[0] = a;
        edge_points[1] = b;
        edge_points[2] = c;

        Vector2r lambda_edge = SignedVolume1D(edge_points[j], edge_points[k]);
        const Vector3r point = edge_points[j] * lambda_edge[0] + edge_points[k] * lambda_edge[1];
        if (Opal::LengthSquared(point) < distance)
        {
            distance = Opal::LengthSquared(point);
            barycentric_coordinates[i] = 0;
            barycentric_coordinates[j] = lambda_edge[0];
            barycentric_coordinates[k] = lambda_edge[1];
        }
    }
    return barycentric_coordinates;
}