#pragma once

#include "opal/container/ref.h"

#include "physics/body.hpp"

namespace Physics
{

bool Intersect(Body& a, Body& b, f32 delta_seconds, struct Contact& contact);
bool RaySphereIntersect(const Vector3r& ray_start, const Vector3r& ray_direction, const Vector3r& sphere_center, real sphere_radius,
                        f32& t1, f32& t2);
bool SphereSphereDynamic(const SphereShape* shape_a, const SphereShape* shape_b, const Vector3r& position_a, const Vector3r& position_b, const Vector3r& velocity_a,
    const Vector3r& velocity_b, f32 delta_seconds, Vector3r& point_on_a, Vector3r& point_on_b, f32& time_of_impact);

}  // namespace Physics