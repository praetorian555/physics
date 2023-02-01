#pragma once

#include "math/base.h"

namespace physics
{
using real = math::real;
}  // namespace physics

// Used to represent a constant of type real
#define PHYSICS_REALC(X) MATH_REALC(X)

#define PHYSICS_UNUSED(X) (void)(X)
