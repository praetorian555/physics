#pragma

#include "math/vector3.h"

#include "physics/base.h"

namespace physics
{

enum class ShapeType
{
    AABox,
    Sphere
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

    [[nodiscard]] bool Overlaps(const Shape& Other) const override;
    [[nodiscard]] real GetSurfaceArea() const override;
    [[nodiscard]] real GetVolume() const override;
};

struct Sphere final : public Shape
{
    math::Vector3 Center;
    real Radius;

    Sphere(const math::Vector3& Center, real Radius);

    [[nodiscard]] bool Overlaps(const Shape& Other) const override;
    [[nodiscard]] real GetSurfaceArea() const override;
    [[nodiscard]] real GetVolume() const override;
};

bool Overlaps(const AABox& A, const AABox& B);
bool Overlaps(const Sphere& A, const Sphere& B);
bool Overlaps(const AABox& A, const Sphere& B);
bool Overlaps(const Sphere& A, const AABox& B);

}  // namespace physics
