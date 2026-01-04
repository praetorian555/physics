#include "physics/gjk.hpp"

Physics::Vector2r Physics::SignedVolume1D(const Vector3r& start, const Vector3r& end)
{
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