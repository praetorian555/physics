#include "physics/shapes/box-shape.hpp"

void Physics::BoxShape::Build(const Opal::ArrayView<Vector3r>& vertices)
{
    m_bounds = Bounds3r(Point3r::Zero());
    for (const auto& vertex : vertices)
    {
        m_bounds = Opal::Union(m_bounds, VectorToPoint(vertex));
    }

    m_vertices.PushBack(Vector3r(m_bounds.min.x, m_bounds.min.y, m_bounds.min.z));
    m_vertices.PushBack(Vector3r(m_bounds.max.x, m_bounds.min.y, m_bounds.min.z));
    m_vertices.PushBack(Vector3r(m_bounds.min.x, m_bounds.max.y, m_bounds.min.z));
    m_vertices.PushBack(Vector3r(m_bounds.min.x, m_bounds.min.y, m_bounds.max.z));

    m_vertices.PushBack(Vector3r(m_bounds.max.x, m_bounds.max.y, m_bounds.max.z));
    m_vertices.PushBack(Vector3r(m_bounds.min.x, m_bounds.max.y, m_bounds.max.z));
    m_vertices.PushBack(Vector3r(m_bounds.max.x, m_bounds.min.y, m_bounds.max.z));
    m_vertices.PushBack(Vector3r(m_bounds.max.x, m_bounds.max.y, m_bounds.min.z));

    m_center_mass = PointToVector((m_bounds.min + PointToVector(m_bounds.max)) * PHYSICS_CONST(0.5));
}

Physics::Matrix3x3r Physics::BoxShape::GetInertiaTensor() const
{
    // Calculate inertia tensor of a box that is centered around zero
    const Vector3r extent = Opal::Extent(m_bounds);
    Matrix3x3r inertia_tensor = Matrix3x3r::Zero();
    inertia_tensor(0, 0) = (extent.y * extent.y + extent.z * extent.z) / PHYSICS_CONST(12.0);
    inertia_tensor(1, 1) = (extent.x * extent.x + extent.z * extent.z) / PHYSICS_CONST(12.0);
    inertia_tensor(2, 2) = (extent.x * extent.x + extent.y * extent.y) / PHYSICS_CONST(12.0);

    // Calculate inertia tensor for a box that is not centered around the origin, using parallel axis theorem
    const Vector3r center_of_mass = PointToVector((m_bounds.min + PointToVector(m_bounds.max)) * PHYSICS_CONST(0.5));
    const Vector3r r = Vector3r::Zero() - center_of_mass;
    const real r_length2 = Opal::LengthSquared(r);
    Matrix3x3r pat_tensor = Matrix3x3r::Zero();
    pat_tensor(0, 0) = r_length2 - (r.x * r.x);
    pat_tensor(0, 1) = r.x * r.y;
    pat_tensor(0, 2) = r.x * r.z;
    pat_tensor(1, 0) = r.x * r.y;
    pat_tensor(1, 1) = r_length2 - (r.y * r.y);
    pat_tensor(1, 2) = r.y * r.z;
    pat_tensor(2, 0) = r.x * r.z;
    pat_tensor(2, 1) = r.y * r.z;
    pat_tensor(2, 2) = r_length2 - (r.z * r.z);

    return inertia_tensor + pat_tensor;
}

Physics::Bounds3r Physics::BoxShape::GetBounds() const
{
    return m_bounds;
}

Physics::Bounds3r Physics::BoxShape::GetBounds(const Vector3r& position, const Quatr& orientation) const
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

Physics::Vector3r Physics::BoxShape::Support(const Vector3r& direction, const Vector3r& position, const Quatr& orientation, f32 bias) const
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

Physics::real Physics::BoxShape::GetFastestLinearSpeed(const Vector3r& angular_velocity, const Vector3r& direction) const
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
