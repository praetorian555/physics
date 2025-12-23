#pragma once

#include "opal/container/ref.h"

#include "physics/body.hpp"

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
    real separation_distance;
    f32 time_of_impact;

    Opal::Ref<Body> a;
    Opal::Ref<Body> b;
};

bool Intersect(Body& a, Body& b, Contact& contact);
void ResolveContact(Contact& contact);

}