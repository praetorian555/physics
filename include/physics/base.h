#pragma once

namespace physics
{
#ifdef PHYSICS_REAL_AS_DOUBLE
// TODO(Marko): To make this work we need to add support for double to math library
using real = double;
#else
using real = float;
#endif
}  // namespace physics

// Used to represent a constant of type real
#ifdef PHYSICS_REAL_AS_DOUBLE
#define PHYSICS_REALC(Value) Value
#else
#define PHYSICS_REALC(Value) Value##f
#endif
