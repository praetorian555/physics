#pragma once

#include "physics/core.hpp"

namespace Physics
{

/**
 * @brief Find barycentric coordinates of the closest point to the origin on a line segment.
 * @param start Start point of the line segment.
 * @param end End point of the line segment.
 * @return Barycentric coordinates (lambda1, lambda2) where closest_point = lambda1 * start + lambda2 * end.
 *         Coordinates always sum to 1. If origin projects outside the segment, returns (1,0) or (0,1).
 * @pre start != end (line segment must have non-zero length).
 * @example
 *     Vector3r start(-1.0f, 0.0f, 0.0f);
 *     Vector3r end(1.0f, 0.0f, 0.0f);
 *     Vector2r bary = SignedVolume1D(start, end);  // Returns (0.5, 0.5)
 *     Vector3r closest = start * bary[0] + end * bary[1];  // (0, 0, 0)
 */
Vector2r SignedVolume1D(const Vector3r& start, const Vector3r& end);

/**
 * @brief Find barycentric coordinates of the closest point to the origin on a triangle.
 * @param a First vertex of the triangle.
 * @param b Second vertex of the triangle.
 * @param c Third vertex of the triangle.
 * @return Barycentric coordinates (lambda_a, lambda_b, lambda_c) where closest_point = lambda_a * a + lambda_b * b + lambda_c * c.
 *         Coordinates always sum to 1. If origin is outside, one or two coordinates will be zero.
 * @pre Triangle must be non-degenerate (vertices must not be collinear).
 * @example
 *     Vector3r a(-1.0f, -1.0f, 0.0f);
 *     Vector3r b(1.0f, -1.0f, 0.0f);
 *     Vector3r c(0.0f, 1.0f, 0.0f);
 *     Vector3r bary = SignedVolume2D(a, b, c);
 *     Vector3r closest = a * bary[0] + b * bary[1] + c * bary[2];
 */
Vector3r SignedVolume2D(const Vector3r& a, const Vector3r& b, const Vector3r& c);

/**
 * @brief Find barycentric coordinates of the closest point to the origin on a tetrahedron.
 * @param a First vertex of the tetrahedron.
 * @param b Second vertex of the tetrahedron.
 * @param c Third vertex of the tetrahedron.
 * @param d Fourth vertex of the tetrahedron.
 * @return Barycentric coordinates (lambda_a, lambda_b, lambda_c, lambda_d) where
 *         closest_point = lambda_a * a + lambda_b * b + lambda_c * c + lambda_d * d.
 *         Coordinates always sum to 1. If origin is outside, one or more coordinates will be zero.
 * @pre Tetrahedron must be non-degenerate (vertices must not be coplanar).
 * @example
 *     Vector3r a(1.0f, 0.0f, 0.0f);
 *     Vector3r b(0.0f, 1.0f, 0.0f);
 *     Vector3r c(0.0f, 0.0f, 1.0f);
 *     Vector3r d(0.0f, 0.0f, 0.0f);
 *     Vector4r bary = SignedVolume3D(a, b, c, d);
 *     Vector3r closest = a * bary[0] + b * bary[1] + c * bary[2] + d * bary[3];
 */
Vector4r SignedVolume3D(const Vector3r& a, const Vector3r& b, const Vector3r& c, const Vector3r& d);

}