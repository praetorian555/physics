#pragma once

#include "opal/container/array-view.h"

#include "physics/core.hpp"

namespace Physics
{

enum class ShapeType : u8
{
    Sphere,
    Box,
    Convex
};

struct Shape
{
    virtual ~Shape() = default;
    virtual void Build(Opal::ArrayView<Vector3r> vertices);
    [[nodiscard]] virtual ShapeType GetType() const = 0;
    [[nodiscard]] virtual Vector3r GetCenterOfMass() const { return m_center_mass; }
    [[nodiscard]] virtual Matrix3x3r GetInertiaTensor() const = 0;
    [[nodiscard]] virtual Bounds3r GetBounds() const = 0;
    [[nodiscard]] virtual Bounds3r GetBounds(const Vector3r& position, const Quatr& orientation) const = 0;
    [[nodiscard]] virtual Vector3r Support(const Vector3r& direction, const Vector3r& position, const Quatr& orientation,
                                           f32 bias) const = 0;
    [[nodiscard]] virtual real GetFastestLinearSpeed(const Vector3r& angular_velocity, const Vector3r& direction) const;

protected:
    Vector3r m_center_mass;
};

}  // namespace Physics