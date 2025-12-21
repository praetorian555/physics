#pragma once

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
};

struct SphereShape : Shape
{
    explicit SphereShape(real radius) : m_radius(radius) {}
    [[nodiscard]] ShapeType GetType() const override { return ShapeType::Sphere; }
    [[nodiscard]] real GetRadius() const { return m_radius; }

private:
    real m_radius = 1.0f;
};

}