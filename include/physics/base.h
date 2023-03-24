#pragma once

#include "math/base.h"
#include "math/point3.h"
#include "math/prettyprint.h"
#include "math/vector3.h"

#if PHYSICS_ENABLE_ASSERTS
#include <cassert>
#define PHYSICS_ASSERT(x) assert(x)
#else
#define PHYSICS_ASSERT(x)
#endif

namespace Physics
{
using real = math::real;
}  // namespace Physics

// Used to represent a constant of type real
#define PHYSICS_REALC(X) MATH_REALC(X)

#define PHYSICS_UNUSED(X) (void)(X)
