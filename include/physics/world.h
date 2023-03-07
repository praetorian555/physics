#pragma once

#include "physics/base.h"
#include "physics/containers.h"
#include "physics/forcegenerator.h"

namespace physics
{

class RigidBody;

/**
 * Represents one simulation of the physics world that contains rigid bodies.
 */
class World
{
public:
    virtual ~World() = default;

    /**
     * Prepares the world for a new simulation frame.
     */
    void StartFrame();

    /**
     * Runs the simulation for the current frame.
     * @param DeltaSeconds The frame duration in seconds.
     */
    void RunSimulation(real DeltaSeconds);

protected:
    /**
     * Integrates the rigid bodies in the world.
     * @param DeltaSeconds The frame duration in seconds.
     */
    void Integrate(real DeltaSeconds);

    /** The rigid bodies in the world. */
    Array<RigidBody*> m_Bodies;

    /** The force registry used to apply forces to the rigid bodies. */
    ForceRegistry m_ForceRegistry;
};

}  // namespace physics
