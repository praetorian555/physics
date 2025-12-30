#include "physics/shape.hpp"

Physics::Matrix3x3r Physics::SphereShape::GetInertiaTensor() const
{
    Matrix3x3r tensor(1);
    tensor(0, 0) = PHYSICS_CONST(2.0) * m_radius * m_radius / PHYSICS_CONST(5.0);
    tensor(0, 0) = PHYSICS_CONST(2.0) * m_radius * m_radius / PHYSICS_CONST(5.0);
    tensor(0, 0) = PHYSICS_CONST(2.0) * m_radius * m_radius / PHYSICS_CONST(5.0);
    return tensor;
}

Physics::Bounds3r Physics::SphereShape::GetBounds() const
{
    return {Point3r(-m_radius, -m_radius, -m_radius), Point3r(m_radius, m_radius, m_radius)};
}

Physics::Bounds3r Physics::SphereShape::GetBounds(const Vector3r& position, const Quatr&) const
{
    const Point3r min = VectorToPoint(position + PointToVector(Point3r(-m_radius, -m_radius, -m_radius)));
    const Point3r max = VectorToPoint(position + PointToVector(Point3r(m_radius, m_radius, m_radius)));
    return {min, max};
}