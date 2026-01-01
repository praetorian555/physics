#pragma once

#include "opal/container/dynamic-array.h"

#include "physics/shape.hpp"

namespace Physics
{

struct BoxShape : Shape
{
    ~BoxShape() override = default;
    void Build(const Opal::ArrayView<Vector3r>& vertices) override;
    [[nodiscard]] ShapeType GetType() const override { return ShapeType::Box; }
    [[nodiscard]] Matrix3x3r GetInertiaTensor() const override;
    [[nodiscard]] Bounds3r GetBounds() const override;
    [[nodiscard]] Bounds3r GetBounds(const Vector3r& position, const Quatr& orientation) const override;
    [[nodiscard]] Vector3r Support(const Vector3r& direction, const Vector3r& position, const Quatr& orientation,
                                           f32 bias) const override;
    [[nodiscard]] real GetFastestLinearSpeed(const Vector3r& angular_velocity, const Vector3r& direction) const override;

private:
    Opal::DynamicArray<Vector3r> m_vertices;
    Bounds3r m_bounds;
};

}  // namespace Physics