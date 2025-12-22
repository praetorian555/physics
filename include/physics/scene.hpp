#pragma once

#include "physics/body.hpp"

namespace Physics
{
class Scene
{
public:
    Scene();
    ~Scene();

    void Update(f32 delta_seconds);

private:
    Opal::DynamicArray<Body> m_bodies;
};
}  // namespace Physics
