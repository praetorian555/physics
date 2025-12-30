#pragma once

#include "opal/container/array-view.h"
#include "opal/container/dynamic-array.h"

#include "physics/body.hpp"
#include "physics/core.hpp"

namespace Physics
{

struct CollisionPair
{
    i32 a;
    i32 b;

    bool operator==(const CollisionPair& other) const;
};

Opal::DynamicArray<CollisionPair> BroadPhase(Opal::ArrayView<Body> bodies, f32 delta_seconds);

}  // namespace Physics