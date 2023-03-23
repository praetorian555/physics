#pragma once

#include "math/base.h"
#include "math/point3.h"
#include "math/prettyprint.h"
#include "math/vector3.h"

#if !NDEBUG
#include <cassert>
#endif

namespace physics
{
using real = math::real;
}  // namespace physics

// Used to represent a constant of type real
#define PHYSICS_REALC(X) MATH_REALC(X)

#define PHYSICS_UNUSED(X) (void)(X)
