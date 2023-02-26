#include "physics/forcegenerator.h"

#include "physics/body.h"

void physics::Gravity::UpdateForce(RigidBody& Body, real DeltaSeconds)
{
    PHYSICS_UNUSED(DeltaSeconds);
    if (!Body.HasFiniteMass())
    {
        return;
    }
    Body.AddForce(m_Gravity * Body.GetMass());
}
