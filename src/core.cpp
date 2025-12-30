#include "physics/core.hpp"

Physics::Point3r Physics::VectorToPoint(const Vector3r& vector)
{
    return {vector.x, vector.y, vector.z};
}

Physics::Vector3r Physics::PointToVector(const Point3r& point)
{
    return {point.x, point.y, point.z};
}
