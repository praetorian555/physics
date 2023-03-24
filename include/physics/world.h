#pragma once

#include "physics/base.h"
#include "physics/containers.h"
#include "physics/force-generator.h"

namespace Physics
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
     * @param delta_seconds The frame duration in seconds.
     */
    void RunSimulation(real delta_seconds);

protected:
    /**
     * Integrates the rigid bodies in the world.
     * @param delta_seconds The frame duration in seconds.
     */
    void Integrate(real delta_seconds);

    /** The rigid bodies in the world. */
    Array<RigidBody*> m_bodies;

    /** The force registry used to apply forces to the rigid bodies. */
    ForceRegistry m_force_registry;
};

}  // namespace Physics
