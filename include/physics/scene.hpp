#pragma once

#include "opal/container/dynamic-array.h"

#include "physics/body.hpp"

namespace Physics
{
class Scene
{
public:
    Scene() = default;
    ~Scene() = default;

    void Update(f32 delta_seconds);
    void AddBody(Body body) { m_bodies.PushBack(body); }
    [[nodiscard]] const Opal::DynamicArray<Body>& GetBodies() const { return m_bodies; }

private:
    Opal::DynamicArray<Body> m_bodies;
    Vector3r m_gravity = Vector3r(0.0, -10.0, 0.0);
};
}  // namespace Physics
