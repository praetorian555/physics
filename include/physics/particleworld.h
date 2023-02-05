#pragma once

#include "physics/containers.h"
#include "physics/particlecontacts.h"
#include "physics/particleforcegenerator.h"

namespace physics
{

class Particle;

class ParticleWorld
{
public:
    // Creates a new particle world.
    // @param MaxContacts - The maximum number of contacts allowed in the world.
    // @param Iterations - The number of iterations to use when resolving contacts. If set to zero
    //                     then the number of iterations will be equal twice the number of
    //                     MaxContacts.
    explicit ParticleWorld(uint32_t MaxContacts, uint32_t Iterations = 0);

    // Initializes the particles for the new frame. This clears the force accumulators for
    // particles.
    void StartFrame();

    // Integrates all the particles.
    // @param DeltaSeconds - The time step.
    void Integrate(real DeltaSeconds);

    // Runs all the contact generators to generate contacts.
    // @return - Returns the number of generated contacts.
    uint32_t GenerateContacts();

    // Processes all the physics for the particle world.
    // @param DeltaSeconds - The time step.
    void Run(real DeltaSeconds);

protected:
    Array<Particle*> m_Particles;
    Array<ParticleContactGenerator*> m_ContactGenerators;
    Array<ParticleContact> m_Contacts;
    ParticleForceRegistry m_ForceRegistry;
    ParticleContactResolver m_ContactResolver;
};

}  // namespace physics