#include "physics/shapes/convex-shape.hpp"

#include "opal/container/in-place-array.h"

namespace
{

Physics::i32 FindPointFurthestInDirection(Opal::ArrayView<Physics::Vector3r> vertices, const Physics::Vector3r& direction)
{
    Physics::i32 idx = 0;
    Physics::real max_distance = static_cast<Physics::real>(Opal::k_neg_inf_double);
    for (Physics::i32 vert_idx = 0; vert_idx < vertices.GetSize(); ++vert_idx)
    {
        const auto& vertex = vertices[vert_idx];
        const Physics::real distance = Opal::Dot(vertex, direction);
        if (distance > max_distance)
        {
            max_distance = distance;
            idx = vert_idx;
        }
    }
    return idx;
}

Physics::real FindDistanceFromLine(const Physics::Vector3r& point, const Physics::Vector3r& a, const Physics::Vector3r& b)
{
    const Physics::Vector3r ab = Opal::Normalize(b - a);
    const Physics::Vector3r ray = point - a;
    const Physics::real distance = Opal::Dot(ray, ab);
    return static_cast<Physics::real>(Opal::Length(ray - distance * ab));
}

Physics::Vector3r FindPointFurthestFromLine(Opal::ArrayView<Physics::Vector3r> vertices, const Physics::Vector3r& a,
                                            const Physics::Vector3r& b)
{
    Physics::i32 idx = 0;
    Physics::real max_distance = 0;
    for (Physics::i32 vert_idx = 0; vert_idx < vertices.GetSize(); ++vert_idx)
    {
        const auto& vertex = vertices[vert_idx];
        const Physics::real distance = FindDistanceFromLine(vertex, a, b);
        if (distance > max_distance)
        {
            max_distance = distance;
            idx = vert_idx;
        }
    }
    return vertices[idx];
}

Physics::real FindDistanceFromTriangle(const Physics::Vector3r& point, const Physics::Vector3r& a, const Physics::Vector3r& b,
                                       const Physics::Vector3r& c)
{
    const Physics::Vector3r ab = b - a;
    const Physics::Vector3r ac = c - a;
    const Physics::Vector3r normal = Opal::Normalize(Opal::Cross(ab, ac));
    return Opal::Dot(normal, point - a);
}

Physics::Vector3r FindPointFurthestFromTriangle(Opal::ArrayView<Physics::Vector3r> vertices, const Physics::Vector3r& a,
                                                const Physics::Vector3r& b, const Physics::Vector3r& c)
{
    Physics::i32 idx = 0;
    Physics::real max_distance = static_cast<Physics::real>(Opal::k_neg_inf_double);
    for (Physics::i32 vert_idx = 0; vert_idx < vertices.GetSize(); ++vert_idx)
    {
        const auto& vertex = vertices[vert_idx];
        const Physics::real distance = FindDistanceFromTriangle(vertex, a, b, c);
        if (distance > max_distance)
        {
            max_distance = distance;
            idx = vert_idx;
        }
    }
    return vertices[idx];
}

void BuildTetrahedron(Opal::ArrayView<Physics::Vector3r> vertices, Opal::DynamicArray<Physics::Vector3r>& out_hull_points,
                      Opal::DynamicArray<Physics::Triangle>& out_triangles)
{
    Opal::InPlaceArray<Physics::Vector3r, 4> points;
    Physics::i32 idx = FindPointFurthestInDirection(vertices, Physics::Vector3r(1, 0, 0));
    points[0] = vertices[idx];
    idx = FindPointFurthestInDirection(vertices, -1 * points[0]);
    points[1] = vertices[idx];
    points[2] = FindPointFurthestFromLine(vertices, points[0], points[1]);
    points[3] = FindPointFurthestFromTriangle(vertices, points[0], points[1], points[2]);

    // Do this check to make sure all faces are CCW.
    const Physics::real distance = FindDistanceFromTriangle(points[3], points[0], points[1], points[2]);
    if (distance > 0)
    {
        std::swap(points[0], points[1]);
    }

    // Append vertices
    out_hull_points.Append(points);

    // Append triangles
    out_triangles.PushBack({.a = 0, .b = 1, .c = 2});
    out_triangles.PushBack({.a = 0, .b = 2, .c = 3});
    out_triangles.PushBack({.a = 2, .b = 1, .c = 3});
    out_triangles.PushBack({.a = 1, .b = 0, .c = 3});
}

void RemoveInternalPoints(Opal::ArrayView<Physics::Vector3r> hull_points, Opal::ArrayView<Physics::Triangle> triangles,
                          Opal::DynamicArray<Physics::Vector3r>& points_to_check)
{
    // Remove any points that are inside the hull
    for (Physics::i32 point_idx = 0; point_idx < points_to_check.GetSize(); ++point_idx)
    {
        const Physics::Vector3r& point = points_to_check[point_idx];
        bool is_external = false;
        for (const Physics::Triangle& triangle : triangles)
        {
            const Physics::Vector3r& a = hull_points[triangle.a];
            const Physics::Vector3r& b = hull_points[triangle.b];
            const Physics::Vector3r& c = hull_points[triangle.c];
            const Physics::real distance = FindDistanceFromTriangle(point, a, b, c);
            if (distance > 0)
            {
                is_external = true;
                break;
            }
        }
        if (!is_external)
        {
            points_to_check.Erase(points_to_check.begin() + point_idx);
            --point_idx;
        }
    }

    // Remove any point that is too close to the hull.
    for (Physics::i32 point_idx = 0; point_idx < points_to_check.GetSize(); ++point_idx)
    {
        const Physics::Vector3r& point = points_to_check[point_idx];
        bool is_too_close = false;
        for (const Physics::Vector3r& hull_point : hull_points)
        {
            constexpr Physics::real k_distance_threshold = PHYSICS_CONST(0.01);  // 1 cm
            if (Opal::LengthSquared(hull_point - point) < k_distance_threshold * k_distance_threshold)
            {
                is_too_close = true;
                break;
            }
        }
        if (is_too_close)
        {
            points_to_check.Erase(points_to_check.begin() + point_idx);
            --point_idx;
        }
    }
}

struct Edge
{
    Physics::i32 a;
    Physics::i32 b;

    bool operator==(const Edge& other) const { return (a == other.a && b == other.b) || (a == other.b && b == other.a); }
};

bool IsEdgeUnique(Opal::ArrayView<Physics::Triangle> triangles, Opal::ArrayView<Physics::i32> facing_triangle_indices,
                  Physics::i32 triangle_index_to_ignore, const Edge& edge)
{
    for (const Physics::i32 facing_triangle_idx : facing_triangle_indices)
    {
        if (facing_triangle_idx == triangle_index_to_ignore)
        {
            continue;
        }
        const Physics::Triangle& triangle = triangles[facing_triangle_idx];
        Opal::InPlaceArray<Edge, 3> edges;
        edges[0].a = triangle.a;
        edges[0].b = triangle.b;
        edges[1].a = triangle.b;
        edges[1].b = triangle.c;
        edges[2].a = triangle.c;
        edges[2].b = triangle.a;
        for (const Edge& triangle_edge : edges)
        {
            if (edge == triangle_edge)
            {
                return false;
            }
        }
    }
    return true;
}

void AddPoint(const Physics::Vector3r& point, Opal::DynamicArray<Physics::Vector3r>& out_hull_points,
              Opal::DynamicArray<Physics::Triangle>& out_triangles)
{
    // Find all triangles facing the given point.
    Opal::DynamicArray<Physics::i32> facing_triangle_indices(Opal::GetScratchAllocator());
    for (Physics::i32 triangle_idx = static_cast<Physics::i32>(out_triangles.GetSize()) - 1; triangle_idx >= 0; --triangle_idx)
    {
        const Physics::Triangle& triangle = out_triangles[triangle_idx];
        const Physics::Vector3r& a = out_hull_points[triangle.a];
        const Physics::Vector3r& b = out_hull_points[triangle.b];
        const Physics::Vector3r& c = out_hull_points[triangle.c];
        const Physics::real distance = FindDistanceFromTriangle(point, a, b, c);
        if (distance > 0)
        {
            facing_triangle_indices.PushBack(triangle_idx);
        }
    }

    // Among the facing triangles find all edges that are unique. These will be used to make new triangles.
    Opal::DynamicArray<Edge> unique_edges(Opal::GetScratchAllocator());
    for (const Physics::i32 facing_triangle_idx : facing_triangle_indices)
    {
        const Physics::Triangle& facing_triangle = out_triangles[facing_triangle_idx];
        Opal::InPlaceArray<Edge, 3> edges;
        edges[0].a = facing_triangle.a;
        edges[0].b = facing_triangle.b;
        edges[1].a = facing_triangle.b;
        edges[1].b = facing_triangle.c;
        edges[2].a = facing_triangle.c;
        edges[2].b = facing_triangle.a;
        for (const Edge& edge : edges)
        {
            if (IsEdgeUnique(out_triangles, facing_triangle_indices, facing_triangle_idx, edge))
            {
                unique_edges.PushBack(edge);
            }
        }
    }

    // Remove all facing indices.
    for (const Physics::i32 facing_triangle_idx : facing_triangle_indices)
    {
        out_triangles.Erase(out_triangles.begin() + facing_triangle_idx);
    }

    out_hull_points.PushBack(point);
    const Physics::i32 new_point_idx = static_cast<Physics::i32>(out_hull_points.GetSize()) - 1;

    // Add new triangles where each one is made out of one unique edge and a new point.
    for (const Edge& unique_edge : unique_edges)
    {
        const Physics::Triangle triangle{.a = unique_edge.a, .b = unique_edge.b, .c = new_point_idx};
        out_triangles.PushBack(triangle);
    }
}

void RemoveUnreferencedVertices(Opal::DynamicArray<Physics::Vector3r>& out_hull_points, Opal::ArrayView<Physics::Triangle> triangles)
{
    for (Physics::i32 point_idx = 0; point_idx < out_hull_points.GetSize(); ++point_idx)
    {
        bool is_point_used = false;
        for (const Physics::Triangle& triangle : triangles)
        {
            if (triangle.a == point_idx || triangle.b == point_idx || triangle.c == point_idx)
            {
                is_point_used = true;
                break;
            }
        }
        if (is_point_used)
        {
            continue;
        }
        for (Physics::Triangle& triangle : triangles)
        {
            if (triangle.a > point_idx)
            {
                --triangle.a;
            }
            if (triangle.b > point_idx)
            {
                --triangle.b;
            }
            if (triangle.c > point_idx)
            {
                --triangle.c;
            }
        }
        out_hull_points.Erase(out_hull_points.begin() + point_idx);
        --point_idx;
    }
}

void ExpandConvexHull(Opal::ArrayView<Physics::Vector3r> vertices, Opal::DynamicArray<Physics::Vector3r>& out_hull_points,
                      Opal::DynamicArray<Physics::Triangle>& out_triangles)
{
    Opal::DynamicArray<Physics::Vector3r> external_vertices(Opal::GetScratchAllocator());
    external_vertices.Append(vertices);

    RemoveInternalPoints(out_hull_points, out_triangles, external_vertices);

    // const int max_steps = 3;
    // int steps = 0;
    while (!external_vertices.IsEmpty())
    {
        const Physics::i32 point_idx = FindPointFurthestInDirection(external_vertices, external_vertices[0]);
        const Physics::Vector3r point = external_vertices[point_idx];
        external_vertices.Erase(external_vertices.begin() + point_idx);
        // This point is outside, now we need to remove old triangles facing it and create new ones
        AddPoint(point, out_hull_points, out_triangles);
        // We added a new point, with new triangles, check again if some of the remaining points are inside the convex hull
        RemoveInternalPoints(out_hull_points, out_triangles, external_vertices);
        // steps++;
    }
    RemoveUnreferencedVertices(out_hull_points, out_triangles);
}

bool IsExternal(Opal::ArrayView<Physics::Vector3r> hull_points, Opal::ArrayView<Physics::Triangle> triangles,
                const Physics::Vector3r& point)
{
    for (const Physics::Triangle& triangle : triangles)
    {
        const Physics::Vector3r& a = hull_points[triangle.a];
        const Physics::Vector3r& b = hull_points[triangle.b];
        const Physics::Vector3r& c = hull_points[triangle.c];
        const Physics::real distance = FindDistanceFromTriangle(point, a, b, c);
        if (distance > 0)
        {
            return true;
        }
    }
    return false;
}

Physics::Vector3r CalculateCenterOfMass(Opal::ArrayView<Physics::Vector3r> hull_points, Opal::ArrayView<Physics::Triangle> triangles,
                                        Physics::i32 sample_count = 100)
{
    Physics::Bounds3r bounds(Physics::Point3r::Zero());
    for (const Physics::Vector3r& point : hull_points)
    {
        bounds = Opal::Union(bounds, Physics::VectorToPoint(point));
    }
    Physics::Vector3r center_of_mass = Physics::Vector3r::Zero();
    Physics::i32 points_inside_count = 0;
    const Physics::Vector3r delta = Opal::Extent(bounds) / sample_count;
    for (Physics::real x = bounds.min.x; x < bounds.max.x; x += delta.x)
    {
        for (Physics::real y = bounds.min.y; y < bounds.max.y; y += delta.y)
        {
            for (Physics::real z = bounds.min.z; z < bounds.max.z; z += delta.z)
            {
                const Physics::Vector3r point(x, y, z);
                if (IsExternal(hull_points, triangles, point))
                {
                    continue;
                }
                center_of_mass += point;
                ++points_inside_count;
            }
        }
    }
    return center_of_mass / static_cast<Physics::real>(points_inside_count);
}

Physics::Matrix3x3r CalculateInertiaTensor(Opal::ArrayView<Physics::Vector3r> hull_points, Opal::ArrayView<Physics::Triangle> triangles,
                                           const Physics::Vector3r& center_mass, Physics::i32 sample_count = 100)
{
    Physics::Bounds3r bounds(Physics::Point3r::Zero());
    for (const Physics::Vector3r& point : hull_points)
    {
        bounds = Opal::Union(bounds, Physics::VectorToPoint(point));
    }
    Physics::Matrix3x3r tensor(0);
    Physics::i32 points_inside_count = 0;
    const Physics::Vector3r delta = Opal::Extent(bounds) / sample_count;
    for (Physics::real x = bounds.min.x; x < bounds.max.x; x += delta.x)
    {
        for (Physics::real y = bounds.min.y; y < bounds.max.y; y += delta.y)
        {
            for (Physics::real z = bounds.min.z; z < bounds.max.z; z += delta.z)
            {
                Physics::Vector3r point(x, y, z);
                if (IsExternal(hull_points, triangles, point))
                {
                    continue;
                }
                point -= center_mass;

                tensor(0, 0) += (point.y * point.y) + (point.z * point.z);
                tensor(1, 1) += (point.x * point.x) + (point.z * point.z);
                tensor(2, 2) += (point.x * point.y) + (point.z * point.z);

                tensor(0, 1) += PHYSICS_CONST(-1) * point.x * point.y;
                tensor(0, 2) += PHYSICS_CONST(-1) * point.x * point.z;
                tensor(1, 2) += PHYSICS_CONST(-1) * point.y * point.z;

                tensor(1, 0) += PHYSICS_CONST(-1) * point.x * point.y;
                tensor(2, 0) += PHYSICS_CONST(-1) * point.x * point.z;
                tensor(2, 1) += PHYSICS_CONST(-1) * point.y * point.z;

                ++points_inside_count;
            }
        }
    }
    tensor *= (PHYSICS_CONST(1) / static_cast<Physics::real>(points_inside_count));
    return tensor;
}

}  // namespace

Physics::ConvexShape::ConvexShape(Opal::ArrayView<Vector3r> vertices)
{
    ConvexShape::Build(vertices);
}

void Physics::ConvexShape::Build(Opal::ArrayView<Vector3r> vertices)
{
    Opal::DynamicArray<Vector3r> hull_points;
    Opal::DynamicArray<Triangle> triangles;
    BuildConvexHull(vertices, hull_points, triangles);
    m_vertices = hull_points;

    m_bounds = Bounds3r(Point3r::Zero());
    for (const Vector3r& vertex : m_vertices)
    {
        m_bounds = Opal::Union(m_bounds, VectorToPoint(vertex));
    }
    m_center_mass = CalculateCenterOfMass(hull_points, triangles, 10);
    m_inertia_tensor = CalculateInertiaTensor(hull_points, triangles, m_center_mass, 10);
}

Physics::Matrix3x3r Physics::ConvexShape::GetInertiaTensor() const
{
    return m_inertia_tensor;
}

Physics::Bounds3r Physics::ConvexShape::GetBounds() const
{
    return m_bounds;
}

Physics::Bounds3r Physics::ConvexShape::GetBounds(const Vector3r& position, const Quatr& orientation) const
{
    Opal::DynamicArray corners(8, Vector3r::Zero(), Opal::GetScratchAllocator());
    corners[0] = Vector3r(m_bounds.min.x, m_bounds.min.y, m_bounds.min.z);
    corners[1] = Vector3r(m_bounds.max.x, m_bounds.min.y, m_bounds.min.z);
    corners[2] = Vector3r(m_bounds.min.x, m_bounds.max.y, m_bounds.min.z);
    corners[3] = Vector3r(m_bounds.min.x, m_bounds.min.y, m_bounds.max.z);
    corners[4] = Vector3r(m_bounds.max.x, m_bounds.max.y, m_bounds.max.z);
    corners[5] = Vector3r(m_bounds.min.x, m_bounds.max.y, m_bounds.max.z);
    corners[6] = Vector3r(m_bounds.max.x, m_bounds.min.y, m_bounds.max.z);
    corners[7] = Vector3r(m_bounds.max.x, m_bounds.max.y, m_bounds.min.z);

    Bounds3r bounds(Point3r::Zero());
    for (auto& corner : corners)
    {
        corner = position + orientation * corner;
        bounds = Opal::Union(bounds, VectorToPoint(corner));
    }
    return bounds;
}

Physics::Vector3r Physics::ConvexShape::Support(const Vector3r& direction, const Vector3r& position, const Quatr& orientation,
                                                f32 bias) const
{
    Vector3r max_point = Vector3r::Zero();
    real max_distance = Opal::k_neg_inf_float;
    for (const auto& vertex : m_vertices)
    {
        const Vector3r& point = position + orientation * vertex;
        const real distance = Opal::Dot(point, direction);
        if (distance > max_distance)
        {
            max_distance = distance;
            max_point = point;
        }
    }
    return max_point + Opal::Normalize(direction) * bias;
}

Physics::real Physics::ConvexShape::GetFastestLinearSpeed(const Vector3r& angular_velocity, const Vector3r& direction) const
{
    real max_speed = PHYSICS_CONST(0.0);
    for (const auto& vertex : m_vertices)
    {
        const Vector3r r = vertex - m_center_mass;
        const Vector3r linear_velocity = Opal::Cross(angular_velocity, r);
        const real speed = Opal::Dot(linear_velocity, direction);
        max_speed = Opal::Max(max_speed, speed);
    }
    return max_speed;
}

void Physics::BuildConvexHull(Opal::ArrayView<Vector3r> vertices, Opal::DynamicArray<Vector3r>& out_hull_points,
                              Opal::DynamicArray<Triangle>& out_triangles)
{
    if (vertices.GetSize() < 4)
    {
        throw Opal::Exception("Not enough vertices to build a convex hull");
    }

    BuildTetrahedron(vertices, out_hull_points, out_triangles);
    ExpandConvexHull(vertices, out_hull_points, out_triangles);
}
