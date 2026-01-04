#pragma once

#include "physics/core.hpp"

namespace Physics
{

/**
 * Find a barycentric coordinates of the origin in reference to line segment in 3D space made by @p start and @p end points.
 */
Vector2r SignedVolume1D(const Vector3r& start, const Vector3r& end);
Vector3r SignedVolume2D(const Vector3r& a, const Vector3r& b, const Vector3r& c);
Vector4r SignedVolume3D(const Vector3r& a, const Vector3r& b, const Vector3r& c, const Vector3r& d);
}