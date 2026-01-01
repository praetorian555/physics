#pragma once

#include "physics/core.hpp"
#include "physics/shape.hpp"

namespace Physics
{

struct SphereShape : Shape
{
    explicit SphereShape(real radius) : m_radius(radius) { m_center_mass = Vector3r::Zero(); }
    [[nodiscard]] ShapeType GetType() const override { return ShapeType::Sphere; }
    [[nodiscard]] Matrix3x3r GetInertiaTensor() const override;
    [[nodiscard]] real GetRadius() const { return m_radius; }
    [[nodiscard]] Bounds3r GetBounds() const override;
    [[nodiscard]] Bounds3r GetBounds(const Vector3r& position, const Quatr& orientation) const override;
    [[nodiscard]] Vector3r Support(const Vector3r& direction, const Vector3r& position, const Quatr& orientation, f32 bias) const override;

private:
    real m_radius = 1.0f;
};

}  // namespace Physics