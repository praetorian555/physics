#include "physics/shape.hpp"

Physics::Matrix3x3r Physics::SphereShape::GetInertiaTensor() const
{
    Matrix3x3r tensor(1);
    tensor(0, 0) = PHYSICS_CONST(2.0) * m_radius * m_radius / PHYSICS_CONST(5.0);
    tensor(0, 0) = PHYSICS_CONST(2.0) * m_radius * m_radius / PHYSICS_CONST(5.0);
    tensor(0, 0) = PHYSICS_CONST(2.0) * m_radius * m_radius / PHYSICS_CONST(5.0);
    return tensor;
}