#include "physics/world.h"

#include "physics/body.h"

void Physics::World::StartFrame()
{
    for (RigidBody* body : m_bodies)
    {
        PHYSICS_ASSERT(body != nullptr);
        body->CalculateDerivedData();
        body->ClearAccumulators();
    }
}

void Physics::World::RunSimulation(Physics::real delta_seconds)
{
    m_force_registry.UpdateForces(delta_seconds);
    Integrate(delta_seconds);
}

void Physics::World::Integrate(Physics::real delta_seconds)
{
    for (RigidBody* body : m_bodies)
    {
        PHYSICS_ASSERT(body != nullptr);
        body->Integrate(delta_seconds);
    }
}
