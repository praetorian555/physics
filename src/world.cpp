#include "physics/world.h"

#include <cassert>

#include "physics/body.h"

void physics::World::StartFrame()
{
    for (RigidBody* Body : m_Bodies)
    {
        assert(Body != nullptr);
        Body->CalculateDerivedData();
        Body->ClearAccumulators();
    }
}

void physics::World::RunSimulation(physics::real DeltaSeconds)
{
    m_ForceRegistry.UpdateForces(DeltaSeconds);
    Integrate(DeltaSeconds);
}

void physics::World::Integrate(physics::real DeltaSeconds)
{
    for (RigidBody* Body : m_Bodies)
    {
        assert(Body != nullptr);
        Body->Integrate(DeltaSeconds);
    }
}
