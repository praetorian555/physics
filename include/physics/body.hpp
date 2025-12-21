#pragma once

#include "physics/core.hpp"
#include "physics/shape.hpp"

#include "rndr/renderers/shape-3d-renderer.hpp"

namespace Physics
{

struct Body
{
    Point3r position;
    Quatr orientation;
    Shape* shape;
};

}  // namespace Physics
