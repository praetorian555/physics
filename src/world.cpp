#include "physics/world.h"

#include "physics/body.h"

void physics::World::StartFrame()
{
    for (RigidBody* body : m_bodies)
    {
        PHYSICS_ASSERT(body != nullptr);
        body->CalculateDerivedData();
        body->ClearAccumulators();
    }
}

void physics::World::RunSimulation(physics::real delta_seconds)
{
    m_force_registry.UpdateForces(delta_seconds);
    Integrate(delta_seconds);
}

void physics::World::Integrate(physics::real delta_seconds)
{
    for (RigidBody* body : m_bodies)
    {
        PHYSICS_ASSERT(body != nullptr);
        body->Integrate(delta_seconds);
    }
}
