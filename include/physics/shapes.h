#pragma

#include "math/vector3.h"

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
    ShapeType Type;

    Shape(ShapeType Type) : Type(Type) {}
    virtual ~Shape() = default;

    /**
     * Checks if this shape is valid. A shape is valid if it has a valid type and all of its members
     * are valid.
     * @return Returns true if the shape is valid, false otherwise.
     */
    [[nodiscard]] virtual bool IsValid() const = 0;

    /**
     * Checks if this shape overlaps with the given shape.
     * @param Other The other shape to check for overlap.
     * @return Returns true if the shapes overlap, false otherwise.
     */
    [[nodiscard]] virtual bool Overlaps(const Shape& Other) const = 0;

    [[nodiscard]] virtual real GetSurfaceArea() const = 0;
    [[nodiscard]] virtual real GetVolume() const = 0;
};

struct AABox : public Shape
{
    math::Vector3 Min;
    math::Vector3 Max;

    AABox();
    AABox(const math::Vector3& Min, const math::Vector3& Max);

    /**
     * Check if the axis-aligned box is valid. An axis-aligned box is valid if the minimum point is
     * less than or equal to the maximum point.
     * @return Returns true if the box is valid, false otherwise.
     */
    [[nodiscard]] bool IsValid() const override;

    [[nodiscard]] bool Overlaps(const Shape& Other) const override;
    [[nodiscard]] real GetSurfaceArea() const override;
    [[nodiscard]] real GetVolume() const override;
};

struct Sphere final : public Shape
{
    math::Vector3 Center;
    real Radius;

    Sphere();
    Sphere(const math::Vector3& Center, real Radius);

    /**
     * Check if the sphere is valid. A sphere is valid if the radius is greater than or equal to 0.
     * @return Returns true if the sphere is valid, false otherwise.
     */
    [[nodiscard]] bool IsValid() const override;

    [[nodiscard]] bool Overlaps(const Shape& Other) const override;
    [[nodiscard]] real GetSurfaceArea() const override;
    [[nodiscard]] real GetVolume() const override;
};

struct Plane : public Shape
{
    /** Normal of the plane. */
    math::Vector3 Normal;
    /** Distance from the origin along the normal. */
    real Distance;

    Plane();
    Plane(const math::Vector3& Normal, real Distance);

    /**
     * Creates a plane from 3 points. Assumes the points are specified in counter-clockwise order
     * and that the points are not collinear.
     * @param A The first point.
     * @param B The second point.
     * @param C The third point.
     * @return Returns the plane. If the points are collinear, the plane will be invalid.
     */
    static Plane FromPoints(const math::Vector3& A, const math::Vector3& B, const math::Vector3& C);

    [[nodiscard]] bool IsValid() const override;
    [[nodiscard]] bool Overlaps(const Shape& Other) const override;
    [[nodiscard]] real GetSurfaceArea() const override { return 0; }
    [[nodiscard]] real GetVolume() const override { return 0; }
};

struct Box : public Shape
{
    math::Vector3 Center;
    math::Vector3 Extents;
    /** The column vector of the rotation matrix that transforms from local box space to world
     * space. */
    union
    {
        struct
        {
            math::Vector3 AxisX;
            math::Vector3 AxisY;
            math::Vector3 AxisZ;
        };
        StackArray<math::Vector3, 3> Axes;
    };

    Box();
    Box(const math::Vector3& Center,
        const math::Vector3& Extents,
        const math::Vector3& AxisX,
        const math::Vector3& AxisY,
        const math::Vector3& AxisZ);
    Box(const math::Vector3& Center,
        const math::Vector3& Extents,
        const math::Matrix4x4& RotationMatrix);

    /**
     * Checks if the box is valid. A box is valid if all extents are larger then Epsilon and the
     * rotation vectors are not zero.
     * @param Epsilon The minimum extent and rotation vector length value.
     * @return Returns true if the box is valid, false otherwise.
     */
    [[nodiscard]] bool IsValid() const override;
    [[nodiscard]] bool Overlaps(const Shape& Other) const override;
    [[nodiscard]] real GetSurfaceArea() const override;
    [[nodiscard]] real GetVolume() const override;
};

/**
 * Checks if the given axis-aligned boxes overlap.
 * @param A The first box.
 * @param B The second box.
 * @return Returns true if the boxes overlap, false otherwise.
 */
bool Overlaps(const AABox& A, const AABox& B);

/**
 * Checks if the given spheres overlap.
 * @param A The first sphere.
 * @param B The second sphere.
 * @return Returns true if the spheres overlap, false otherwise.
 */
bool Overlaps(const Sphere& A, const Sphere& B);

/**
 * Checks if the given planes overlap.
 * @param A The first plane.
 * @param B The second plane.
 * @return Returns true if the planes overlap, false otherwise.
 */
bool Overlaps(const Plane& A, const Plane& B);

/**
 * Checks if the given boxes overlap.
 * @param A The first box.
 * @param B The second box.
 * @return Returns true if the boxes overlap, false otherwise.
 */
bool Overlaps(const Box& A, const Box& B);

/**
 * Checks if the given sphere overlaps with the given axis-aligned box.
 * @param A The sphere.
 * @param B The box.
 * @return Returns true if the sphere overlaps with the box, false otherwise.
 */
bool Overlaps(const AABox& A, const Sphere& B);
bool Overlaps(const Sphere& A, const AABox& B);

/**
 * Checks if the given plane overlaps with the given axis-aligned box.
 * @param A The plane.
 * @param B The box.
 * @return Returns true if the plane overlaps with the box, false otherwise.
 */
bool Overlaps(const AABox& A, const Plane& B);
bool Overlaps(const Plane& A, const AABox& B);

/**
 * Checks if the given plane overlaps with the given sphere.
 * @param A The plane.
 * @param B The sphere.
 * @return Returns true if the plane overlaps with the sphere, false otherwise.
 */
bool Overlaps(const Sphere& A, const Plane& B);
bool Overlaps(const Plane& A, const Sphere& B);

/**
 * Checks if the given box overlaps with the given axis-aligned box.
 * @param A The box.
 * @param B The axis-aligned box.
 * @return Returns true if the box overlaps with the axis-aligned box, false otherwise.
 */
bool Overlaps(const Box& A, const AABox& B);
bool Overlaps(const AABox& A, const Box& B);

/**
 * Checks if the given box overlaps with the given sphere.
 * @param A The box.
 * @param B The sphere.
 * @return Returns true if the box overlaps with the sphere, false otherwise.
 */
bool Overlaps(const Box& A, const Sphere& B);
bool Overlaps(const Sphere& A, const Box& B);

/**
 * Checks if the given box overlaps with the given plane.
 * @param A The box.
 * @param B The plane.
 * @return Returns true if the box overlaps with the plane, false otherwise.
 */
bool Overlaps(const Box& A, const Plane& B);
bool Overlaps(const Plane& A, const Box& B);

/**
 * Find the closest point on the plane to the given point.
 * @param Point Point to find the closest point on the plane to.
 * @param Plane Plane to find the closest point on.
 * @return Returns the closest point on the plane to the given point.
 */
math::Vector3 ClosestPoint(const math::Vector3& Point, const Plane& Plane);

/**
 * Find the closest point on the axis-aligned box to the given point.
 * @param Point Point to find the closest point on the box to.
 * @param Box Axis-aligned box to find the closest point on.
 * @return Returns the closest point on the axis-aligned box to the given point.
 */
math::Vector3 ClosestPoint(const math::Vector3& Point, const AABox& Box);

/**
 * Find the closest point on the sphere to the given point.
 * @param Point Point to find the closest point on the sphere to.
 * @param Sphere Sphere to find the closest point on.
 * @return Returns the closest point on the sphere to the given point.
 */
math::Vector3 ClosestPoint(const math::Vector3& Point, const Sphere& Sphere);

/**
 * Find the closest point on the box to the given point.
 * @param Point Point to find the closest point on the box to.
 * @param Box Box to find the closest point on.
 * @return Returns the closest point on the box to the given point.
 */
math::Vector3 ClosestPoint(const math::Vector3& Point, const Box& Box);

/**
 * Find the distance from the given point to the plane.
 * @param Point Point to find the distance to the plane.
 * @param Plane Plane to find the distance to.
 * @return Returns the distance from the point to the plane. The distance is negative if the point
 * is on the negative side of the plane.
 */
real Distance(const math::Vector3& Point, const Plane& Plane);

/**
 * Find the distance from the given point to the axis-aligned box.
 * @param Point Point to find the distance to the box.
 * @param Box Axis-aligned box to find the distance to.
 * @return Returns the distance from the point to the axis-aligned box.
 */
real Distance(const math::Vector3& Point, const AABox& Box);

/**
 * Find the distance from the given point to the sphere.
 * @param Point Point to find the distance to the sphere.
 * @param Sphere Sphere to find the distance to.
 * @return Returns the distance from the point to the sphere.
 */
real Distance(const math::Vector3& Point, const Sphere& Sphere);

/**
 * Find the distance from the given point to the box.
 * @param Point Point to find the distance to the box.
 * @param Box Box to find the distance to.
 * @return Returns the distance from the point to the box.
 */
real Distance(const math::Vector3& Point, const Box& Box);

/**
 * Find the square distance from the given point to the axis-aligned box.
 * @param Point Point to find the square distance to the box.
 * @param Box Axis-aligned box to find the square distance to.
 * @return Returns the square distance from the point to the axis-aligned box.
 */
real SquareDistance(const math::Vector3& Point, const AABox& Box);

}  // namespace physics
