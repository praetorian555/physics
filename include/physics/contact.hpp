#pragma once

#include "opal/container/ref.h"

#include "physics/body.hpp"
#include "physics/core.hpp"

namespace Physics
{

struct Contact
{
    Vector3r point_on_a_world_space;
    Vector3r point_on_b_world_space;
    Vector3r point_on_a_local_space;
    Vector3r point_on_b_local_space;

    // In world space
    Vector3r normal;
    // Positive when non-penetrating, negative when penetrating
    real separation_distance = PHYSICS_CONST(0.0);
    f32 time_of_impact = 0.0f;

    Opal::Ref<Body> a;
    Opal::Ref<Body> b;

#ifdef OPAL_DEBUG
    Vector3r a_prev_position;
    Vector3r b_prev_position;
    Vector3r a_prev_linear_velocity;
    Vector3r b_prev_linear_velocity;
#endif
};

void ResolveContact(Contact& contact);

}  // namespace Physics