#pragma once

#include "physics/base.h"
#include "physics/containers.h"

namespace math
{
struct Matrix4x4;
}

namespace physics
{

enum class ShapeType
{
    AABox = 0,
    Sphere,
    Plane,
    Box,
    Max
};

/**
 * Represents a shape that can be used to detect collisions. Abstract class.
 */
struct Shape
{
    ShapeType type;

    Shape(ShapeType in_type) : type(in_type) {}
    virtual ~Shape() = default;

    /**
     * Checks if this shape is valid. A shape is valid if it has a valid type and all of its members
     * are valid.
     * @return Returns true if the shape is valid, false otherwise.
     */
    [[nodiscard]] virtual bool IsValid() const = 0;

    /**
     * Checks if this shape overlaps with the given shape.
     * @param other The other shape to check for overlap.
     * @return Returns true if the shapes overlap, false otherwise.
     */
    [[nodiscard]] virtual bool Overlaps(const Shape& other) const = 0;

    [[nodiscard]] virtual real GetSurfaceArea() const = 0;
    [[nodiscard]] virtual real GetVolume() const = 0;
};

struct AABox : public Shape
{
    math::Point3 min;
    math::Point3 max;

    AABox();
    AABox(const math::Point3& in_min, const math::Point3& in_max);

    /**
     * Check if the axis-aligned box is valid. An axis-aligned box is valid if the minimum point is
     * less than or equal to the maximum point.
     * @return Returns true if the box is valid, false otherwise.
     */
    [[nodiscard]] bool IsValid() const override;

    [[nodiscard]] bool Overlaps(const Shape& other) const override;
    [[nodiscard]] real GetSurfaceArea() const override;
    [[nodiscard]] real GetVolume() const override;
};

struct Sphere final : public Shape
{
    math::Point3 center;
    real radius;

    Sphere();
    Sphere(const math::Point3& center, real radius);

    /**
     * Check if the sphere is valid. A sphere is valid if the radius is greater than or equal to 0.
     * @return Returns true if the sphere is valid, false otherwise.
     */
    [[nodiscard]] bool IsValid() const override;

    [[nodiscard]] bool Overlaps(const Shape& other) const override;
    [[nodiscard]] real GetSurfaceArea() const override;
    [[nodiscard]] real GetVolume() const override;
};

// TODO(Marko): Make normal be of type math::Normal.
struct Plane : public Shape
{
    /** Normal of the plane. */
    math::Vector3 normal;
    /** Distance from the origin along the normal. */
    real distance;

    Plane();
    Plane(const math::Vector3& in_normal, real in_distance);

    /**
     * Creates a plane from 3 points. Assumes the points are specified in counter-clockwise order
     * and that the points are not collinear.
     * @param a The first point.
     * @param b The second point.
     * @param c The third point.
     * @return Returns the plane. If the points are collinear, the plane will be invalid.
     */
    static Plane FromPoints(const math::Point3& a, const math::Point3& b, const math::Point3& c);

    [[nodiscard]] bool IsValid() const override;
    [[nodiscard]] bool Overlaps(const Shape& other) const override;
    [[nodiscard]] real GetSurfaceArea() const override { return 0; }
    [[nodiscard]] real GetVolume() const override { return 0; }
};

struct Box : public Shape
{
    math::Point3 center;
    math::Vector3 half_extents;
    /**
     * The column vectors of the rotation matrix that transforms from local box space to world
     * space.
     */
    union
    {
        struct
        {
            math::Vector3 axis_x;
            math::Vector3 axis_y;
            math::Vector3 axis_z;
        };
        StackArray<math::Vector3, 3> axes;
    };

    Box();
    Box(const math::Point3& in_center,
        const math::Vector3& in_half_extents,
        const math::Vector3& in_axis_x,
        const math::Vector3& in_axis_y,
        const math::Vector3& in_axis_z);
    Box(const math::Point3& in_center,
        const math::Vector3& in_half_extents,
        const math::Matrix4x4& in_rotation_matrix);

    /**
     * Checks if the box is valid. A box is valid if all extents are larger then Epsilon and the
     * rotation vectors are not zero.
     * @param Epsilon The minimum extent and rotation vector length value.
     * @return Returns true if the box is valid, false otherwise.
     */
    [[nodiscard]] bool IsValid() const override;
    [[nodiscard]] bool Overlaps(const Shape& other) const override;
    [[nodiscard]] real GetSurfaceArea() const override;
    [[nodiscard]] real GetVolume() const override;
};

}  // namespace physics
