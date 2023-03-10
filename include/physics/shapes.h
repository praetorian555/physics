#pragma

#include "math/vector3.h"

#include "physics/base.h"

namespace physics
{

enum class ShapeType
{
    AABox = 0,
    Sphere,
    Plane,
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
     * @return Returns the plane.
     */
    static Plane FromPoints(const math::Vector3& A, const math::Vector3& B, const math::Vector3& C);

    [[nodiscard]] bool IsValid() const override;
    [[nodiscard]] bool Overlaps(const Shape& Other) const override;
    [[nodiscard]] real GetSurfaceArea() const override { return 0; }
    [[nodiscard]] real GetVolume() const override { return 0; }
};

bool Overlaps(const AABox& A, const AABox& B);
bool Overlaps(const Sphere& A, const Sphere& B);
bool Overlaps(const Plane& A, const Plane& B);
bool Overlaps(const AABox& A, const Sphere& B);
bool Overlaps(const Sphere& A, const AABox& B);
bool Overlaps(const AABox& A, const Plane& B);
bool Overlaps(const Plane& A, const AABox& B);
bool Overlaps(const Sphere& A, const Plane& B);
bool Overlaps(const Plane& A, const Sphere& B);

}  // namespace physics
