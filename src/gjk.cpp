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

/**
 * @brief Check whether two scalars share the same sign, treating zero as matching either sign.
 * @param a First scalar.
 * @param b Second scalar.
 * @return True if both values are non-negative or both are non-positive, false otherwise.
 * @example
 *     bool same = IsSameSign(-2.0f, -0.5f);  // true
 *     bool diff = IsSameSign(-1.0f, 1.0f);   // false
 */
bool IsSameSign(Physics::real a, Physics::real b)
{
    return (a >= 0 && b >= 0) || (a <= 0 && b <= 0);
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
    real distance = PHYSICS_CONST(1e10);
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

Physics::Vector4r Physics::SignedVolume3D(const Vector3r& a, const Vector3r& b, const Vector3r& c, const Vector3r& d)
{
    const Matrix4x4r m = Matrix4x4r::FromRows({a.x, b.x, c.x, d.x}, {a.y, b.y, c.y, d.y}, {a.z, b.z, c.z, d.z}, {1, 1, 1, 1});

    Vector4r cofactors;
    cofactors[0] = Opal::Cofactor(m, 3, 0);
    cofactors[1] = Opal::Cofactor(m, 3, 1);
    cofactors[2] = Opal::Cofactor(m, 3, 2);
    cofactors[3] = Opal::Cofactor(m, 3, 3);

    const real m_det = cofactors[0] + cofactors[1] + cofactors[2] + cofactors[3];
    PHYSICS_ASSERT(!Opal::IsEqual(m_det, PHYSICS_CONST(0), PHYSICS_CONST(0.0000001)), "Determinant can't be zero");

    if (IsSameSign(m_det, cofactors[0]) && IsSameSign(m_det, cofactors[1]) && IsSameSign(m_det, cofactors[2]) &&
        IsSameSign(m_det, cofactors[3]))
    {
        return cofactors / m_det;
    }

    // If we are here, we need to project the origin onto the faces and determine the closest point.
    real distance = PHYSICS_CONST(1e10);
    Vector4r barycentric_coordinates = Vector4r::Zero();
    for (i32 i = 0; i < 4; i++)
    {
        const i32 j = (i + 1) % 4;
        const i32 k = (i + 2) % 4;

        Opal::InPlaceArray<Vector3r, 4> face_points;
        face_points[0] = a;
        face_points[1] = b;
        face_points[2] = c;
        face_points[3] = d;

        const Vector3r lambdas = SignedVolume2D(face_points[i], face_points[j], face_points[k]);
        const Vector3r point = face_points[i] * lambdas[0] + face_points[j] * lambdas[1] + face_points[k] * lambdas[2];
        if (Opal::LengthSquared(point) < distance)
        {
            distance = Opal::LengthSquared(point);
            barycentric_coordinates[i] = lambdas[0];
            barycentric_coordinates[j] = lambdas[1];
            barycentric_coordinates[k] = lambdas[2];
        }
    }
    return barycentric_coordinates;
}

Physics::Point Physics::Support(const Body& body_a, const Body& body_b, const Vector3r& direction, const real bias)
{
    PHYSICS_ASSERT(body_a.shape != nullptr, "Shape of body A cannot be null");
    PHYSICS_ASSERT(body_b.shape != nullptr, "Shape of body B cannot be null");
    PHYSICS_ASSERT(Opal::LengthSquared(direction) > 0.0f, "Direction vector cannot be zero");

    Point out;
    const Vector3r normalized_direction = Opal::Normalize(direction);
    out.point_on_body_a = body_a.shape->Support(normalized_direction, body_a.position, body_b.orientation, bias);
    out.point_on_body_b = body_b.shape->Support(-1 * normalized_direction, body_b.position, body_b.orientation, bias);
    out.point_on_mink_diff = out.point_on_body_a - out.point_on_body_b;
    return out;
}

namespace
{
/**
 * @brief Check whether a candidate support point is already present in the simplex.
 * @param simplex_points Current simplex points; only their `point_on_mink_diff` is compared.
 * @param new_point Candidate support point to test for duplication.
 * @return True if any existing simplex point lies within 1e-6 (squared distance) of new_point on the
 *         Minkowski difference, false otherwise.
 * @example
 *     if (HasPoint(simplex_points, candidate)) { break; }  // GJK has converged.
 */
bool HasPoint(const Opal::InPlaceArray<Physics::Point, 4>& simplex_points, const Physics::Point& new_point)
{
    constexpr Physics::real k_precision = PHYSICS_CONST(1e-6);
    for (Physics::i32 i = 0; i < 4; i++)
    {
        const Physics::Vector3r delta = simplex_points[i].point_on_mink_diff - new_point.point_on_mink_diff;
        if (Opal::LengthSquared(delta) < k_precision * k_precision)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Reduce a GJK simplex to its sub-simplex closest to the origin and compute the next search direction.
 * @param simplex_points Current simplex (2, 3, or 4 points) on the Minkowski difference.
 * @param num_points Number of valid entries in simplex_points; must be 2, 3, or 4.
 * @param out_new_direction Output direction from the simplex's closest feature toward the origin.
 * @param out_lambdas Output barycentric weights of the origin's projection; entries past num_points are unset.
 * @return True if the origin coincides with the simplex's closest point (squared distance < 1e-8),
 *         indicating intersection; false otherwise.
 * @pre num_points must be 2, 3, or 4. Cases outside this range leave outputs unset.
 * @example
 *     Vector3r dir; Vector4r lambdas;
 *     if (SimplexSignedVolumes(simplex_points, num_points, dir, lambdas)) {
 *         // Origin is inside the simplex; shapes intersect.
 *     }
 */
bool SimplexSignedVolumes(const Opal::InPlaceArray<Physics::Point, 4>& simplex_points, Physics::i32 num_points,
                          Physics::Vector3r& out_new_direction, Physics::Vector4r& out_lambdas)
{
    PHYSICS_ASSERT((num_points == 2) || (num_points == 3) || (num_points == 4), "Number of points must be 2, 3 or 4");

    using namespace Physics;
    constexpr real k_epsilon_sq = PHYSICS_CONST(1e-4) * PHYSICS_CONST(1e-4);
    bool does_intersect = false;
    switch (num_points)
    {
        case 2:
        {
            Vector2r lambdas = SignedVolume1D(simplex_points[0].point_on_mink_diff, simplex_points[1].point_on_mink_diff);
            out_new_direction = Vector3r::Zero();
            for (i32 i = 0; i < num_points; i++)
            {
                out_new_direction += simplex_points[i].point_on_mink_diff * lambdas[i];
            }
            // Point towards the origin
            out_new_direction *= -1;
            does_intersect = Opal::LengthSquared(out_new_direction) < k_epsilon_sq;
            out_lambdas[0] = lambdas[0];
            out_lambdas[1] = lambdas[1];
            break;
        }
        case 3:
        {
            Vector3r lambdas = SignedVolume2D(simplex_points[0].point_on_mink_diff, simplex_points[1].point_on_mink_diff,
                                              simplex_points[2].point_on_mink_diff);
            out_new_direction = Vector3r::Zero();
            for (i32 i = 0; i < num_points; i++)
            {
                out_new_direction += simplex_points[i].point_on_mink_diff * lambdas[i];
            }
            // Point towards the origin
            out_new_direction *= -1;
            does_intersect = Opal::LengthSquared(out_new_direction) < k_epsilon_sq;
            out_lambdas[0] = lambdas[0];
            out_lambdas[1] = lambdas[1];
            out_lambdas[2] = lambdas[2];
            break;
        }
        case 4:
        {
            Vector4r lambdas = SignedVolume3D(simplex_points[0].point_on_mink_diff, simplex_points[1].point_on_mink_diff,
                                              simplex_points[2].point_on_mink_diff, simplex_points[3].point_on_mink_diff);
            out_new_direction = Vector3r::Zero();
            for (i32 i = 0; i < num_points; i++)
            {
                out_new_direction += simplex_points[i].point_on_mink_diff * lambdas[i];
            }
            // Point towards the origin
            out_new_direction *= -1;
            does_intersect = Opal::LengthSquared(out_new_direction) < k_epsilon_sq;
            out_lambdas[0] = lambdas[0];
            out_lambdas[1] = lambdas[1];
            out_lambdas[2] = lambdas[2];
            out_lambdas[3] = lambdas[3];
            break;
        }
    }
    return does_intersect;
}

/**
 * @brief Compact a simplex by removing points whose barycentric weight is exactly zero.
 * @param in_out_points Simplex points; surviving entries are moved to the front, the rest zeroed.
 * @param in_out_lambdas Matching barycentric weights; surviving entries are moved to the front, the rest set to zero.
 * @return Number of surviving points (those with non-zero lambda), in the range [0, 4].
 * @example
 *     // After SimplexSignedVolumes drops a vertex (lambda == 0), shrink the simplex:
 *     const i32 num_points = SortValids(simplex_points, lambdas);
 */
Physics::i32 SortValids(Opal::InPlaceArray<Physics::Point, 4>& in_out_points, Physics::Vector4r& in_out_lambdas)
{
    using namespace Physics;
    bool valids[4] = {false, false, false, false};
    for (i32 i = 0; i < 4; i++)
    {
        valids[i] = true;
        if (in_out_lambdas[i] == 0.0f)
        {
            valids[i] = false;
        }
    }

    Vector4r valid_lambdas = Vector4r::Zero();
    i32 valid_count = 0;
    Opal::InPlaceArray<Point, 4> valid_points;
    memset(&valid_points, 0, sizeof(valid_points));
    for (i32 i = 0; i < 4; i++)
    {
        if (valids[i])
        {
            valid_points[valid_count] = in_out_points[i];
            valid_lambdas[valid_count] = in_out_lambdas[i];
            valid_count++;
        }
    }

    for (int i = 0; i < 4; i++)
    {
        in_out_points[i] = valid_points[i];
        in_out_lambdas[i] = valid_lambdas[i];
    }
    return valid_count;
}

}  // namespace

bool Physics::IntersectGJK(const Body& body_a, const Body& body_b)
{
    // GJK intersection test.
    //
    // Two convex shapes A and B intersect iff their Minkowski difference A - B = { a - b : a in A, b in B }
    // contains the origin. GJK searches for the origin inside this set without ever building it explicitly,
    // using only the support function (the point on A - B the furthest along a query direction).
    //
    // The algorithm maintains a simplex (1 to 4 points) of support points on A - B and iteratively refines it:
    //   1. Seed the simplex with a support point in an arbitrary direction. The next search direction is the
    //      vector from that point toward the origin.
    //   2. Each iteration, query a new support point in the current direction. If it is not strictly past the
    //      origin along that direction (dot < 0), the origin lies outside A - B and the shapes are disjoint.
    //   3. Add the new point to the simplex and reduce it to the sub-simplex closest to the origin via
    //      SimplexSignedVolumes. That routine returns the barycentric weights of the origin's projection
    //      onto the simplex's closest feature and the next search direction (from that feature toward the origin).
    //   4. If the search direction's squared length stops decreasing, or the new support is a duplicate, we
    //      have converged without enclosing the origin: the shapes are disjoint.
    //   5. SortValids compacts the simplex by dropping vertices whose barycentric weight is zero (they lie
    //      outside the closest feature) and returns the surviving count. If all four points survive, the
    //      origin is enclosed by the tetrahedron and the shapes intersect.
    //
    // Termination: each iteration either encloses the origin, proves separation, or strictly decreases the
    // distance from the simplex to the origin, so the loop is finite for non-degenerate convex inputs.

    PHYSICS_ASSERT(body_a.shape != nullptr, "Shape of body A cannot be null");
    PHYSICS_ASSERT(body_b.shape != nullptr, "Shape of body B cannot be null");

    constexpr Vector3r k_origin(0, 0, 0);
    i32 num_points = 1;
    Opal::InPlaceArray<Point, 4> simplex_points;
    real closest_distance = Opal::k_inf_float;
    bool does_contain_origin = false;

    simplex_points[0] = Support(body_a, body_b, Vector3r(1, 1, 1), 0);
    Vector3r new_direction = -1 * simplex_points[0].point_on_mink_diff;
    do
    {
        const Point new_point = Support(body_a, body_b, new_direction, 0);
        if (HasPoint(simplex_points, new_point))
        {
            break;
        }
        simplex_points[num_points++] = new_point;

        // If this new point hasn't moved pass the origin, then the origin can't be in the set, so no collision.
        if (Opal::Dot(new_direction, new_point.point_on_mink_diff - k_origin) < 0)
        {
            break;
        }

        Vector4r lambdas;
        does_contain_origin = SimplexSignedVolumes(simplex_points, num_points, new_direction, lambdas);
        if (does_contain_origin)
        {
            break;
        }

        // If we are not getting closer to the origin there is no intersection so exit early
        const real distance = Opal::LengthSquared(new_direction);
        if (distance >= closest_distance)
        {
            break;
        }
        closest_distance = distance;

        num_points = SortValids(simplex_points, lambdas);
        does_contain_origin = (num_points == 4);
    } while (!does_contain_origin);
    return does_contain_origin;
}
