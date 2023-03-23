#pragma once

#include "physics/base.h"

namespace physics
{

struct AABox;
struct Sphere;
struct Plane;
struct Box;

/**
 * Checks if the given axis-aligned boxes overlap.
 * @param a The first box.
 * @param b The second box.
 * @return Returns true if the boxes overlap, false otherwise.
 */
bool Overlaps(const AABox& a, const AABox& b);

/**
 * Checks if the given spheres overlap.
 * @param a The first sphere.
 * @param b The second sphere.
 * @return Returns true if the spheres overlap, false otherwise.
 */
bool Overlaps(const Sphere& a, const Sphere& b);

/**
 * Checks if the given planes overlap.
 * @param a The first plane.
 * @param b The second plane.
 * @return Returns true if the planes overlap, false otherwise.
 */
bool Overlaps(const Plane& a, const Plane& b);

/**
 * Checks if the given boxes overlap.
 * @param a The first box.
 * @param b The second box.
 * @return Returns true if the boxes overlap, false otherwise.
 */
bool Overlaps(const Box& a, const Box& b);

/**
 * Checks if the given sphere overlaps with the given axis-aligned box.
 * @param a The sphere.
 * @param b The box.
 * @return Returns true if the sphere overlaps with the box, false otherwise.
 */
bool Overlaps(const AABox& a, const Sphere& b);
bool Overlaps(const Sphere& a, const AABox& b);

/**
 * Checks if the given plane overlaps with the given axis-aligned box.
 * @param a The plane.
 * @param b The box.
 * @return Returns true if the plane overlaps with the box, false otherwise.
 */
bool Overlaps(const AABox& a, const Plane& b);
bool Overlaps(const Plane& a, const AABox& b);

/**
 * Checks if the given plane overlaps with the given sphere.
 * @param a The plane.
 * @param b The sphere.
 * @return Returns true if the plane overlaps with the sphere, false otherwise.
 */
bool Overlaps(const Sphere& a, const Plane& b);
bool Overlaps(const Plane& a, const Sphere& b);

/**
 * Checks if the given box overlaps with the given axis-aligned box.
 * @param a The box.
 * @param b The axis-aligned box.
 * @return Returns true if the box overlaps with the axis-aligned box, false otherwise.
 */
bool Overlaps(const Box& a, const AABox& b);
bool Overlaps(const AABox& a, const Box& b);

/**
 * Checks if the given box overlaps with the given sphere.
 * @param a The box.
 * @param b The sphere.
 * @return Returns true if the box overlaps with the sphere, false otherwise.
 */
bool Overlaps(const Box& a, const Sphere& b);
bool Overlaps(const Sphere& a, const Box& b);

/**
 * Checks if the given box overlaps with the given plane.
 * @param a The box.
 * @param b The plane.
 * @return Returns true if the box overlaps with the plane, false otherwise.
 */
bool Overlaps(const Box& a, const Plane& b);
bool Overlaps(const Plane& a, const Box& b);

}  // namespace physics
