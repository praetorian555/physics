#pragma once

#include "opal/math/point2.h"
#include "physics/core.hpp"

namespace Physics
{

enum class ShapeType : u8
{
    Sphere
};

struct Shape
{
    virtual ~Shape() = default;
    [[nodiscard]] virtual ShapeType GetType() const = 0;
    [[nodiscard]] virtual Vector3r GetCenterOfMass() const { return m_center_mass; }
    [[nodiscard]] virtual Matrix3x3r GetInertiaTensor() const = 0;
    [[nodiscard]] virtual Bounds3r GetBounds() const = 0;
    [[nodiscard]] virtual Bounds3r GetBounds(const Vector3r& position, const Quatr& orientation) const = 0;


protected:
    Vector3r m_center_mass;
};

struct SphereShape : Shape
{
    explicit SphereShape(real radius) : m_radius(radius) { m_center_mass = Vector3r::Zero(); }
    [[nodiscard]] ShapeType GetType() const override { return ShapeType::Sphere; }
    [[nodiscard]] Matrix3x3r GetInertiaTensor() const override;
    [[nodiscard]] real GetRadius() const { return m_radius; }
    [[nodiscard]] Bounds3r GetBounds() const override;
    [[nodiscard]] Bounds3r GetBounds(const Vector3r& position, const Quatr& orientation) const override;

private:
    real m_radius = 1.0f;
};

}  // namespace Physics