#pragma once

#include "physics/base.h"

namespace physics
{

struct AABox;
struct Sphere;
struct Plane;
struct Box;

/**
 * Find the closest point on the plane to the given point.
 * @param point A point to find the closest point on the plane to.
 * @param plane A plane to find the closest point on.
 * @return Returns the closest point on the plane to the given point.
 */
math::Point3 ClosestPoint(const math::Point3& point, const Plane& plane);

/**
 * Find the closest point on the axis-aligned box to the given point.
 * @param point A point to find the closest point on the box to.
 * @param box An axis-aligned box to find the closest point on.
 * @return Returns the closest point on the axis-aligned box to the given point.
 */
math::Point3 ClosestPoint(const math::Point3& point, const AABox& box);

/**
 * Find the closest point on the sphere to the given point.
 * @param point A point to find the closest point on the sphere to.
 * @param sphere A sphere to find the closest point on.
 * @return Returns the closest point on the sphere to the given point.
 */
math::Point3 ClosestPoint(const math::Point3& point, const Sphere& sphere);

/**
 * Find the closest point on the box to the given point.
 * @param point A point to find the closest point on the box to.
 * @param box A box to find the closest point on.
 * @return Returns the closest point on the box to the given point.
 */
math::Point3 ClosestPoint(const math::Point3& point, const Box& box);

/**
 * Find the distance from the given point to the plane.
 * @param point A point to find the distance to the plane.
 * @param plane A plane to find the distance to.
 * @return Returns the distance from the point to the plane. The distance is negative if the point
 * is on the negative side of the plane.
 */
real Distance(const math::Point3& point, const Plane& plane);

/**
 * Find the distance from the given point to the axis-aligned box.
 * @param point A point to find the distance to the box.
 * @param box An axis-aligned box to find the distance to.
 * @return Returns the distance from the point to the axis-aligned box.
 */
real Distance(const math::Point3& point, const AABox& box);

/**
 * Find the distance from the given point to the sphere.
 * @param point A point to find the distance to the sphere.
 * @param sphere A sphere to find the distance to.
 * @return Returns the distance from the point to the sphere.
 */
real Distance(const math::Point3& point, const Sphere& sphere);

/**
 * Find the distance from the given point to the box.
 * @param point A point to find the distance to the box.
 * @param box A box to find the distance to.
 * @return Returns the distance from the point to the box.
 */
real Distance(const math::Point3& point, const Box& box);

/**
 * Find the square distance from the given point to the axis-aligned box.
 * @param point A point to find the square distance to the box.
 * @param box An axis-aligned box to find the square distance to.
 * @return Returns the square distance from the point to the axis-aligned box.
 */
real SquareDistance(const math::Point3& point, const AABox& box);

/**
 * Enclose the two input axis-aligned boxes within a new, smallest possible, axis-aligned box and
 * store the result in the out_box parameter.
 * @param out_box The output box.
 * @param in_box0 The first input box.
 * @param in_box1 The second input box.
 */
void Enclose(AABox& out_box, const AABox& in_box0, const AABox& in_box1);

}  // namespace physics
