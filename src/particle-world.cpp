#include "physics/particle-world.h"

#include "physics/particle.h"

Physics::ParticleWorld::ParticleWorld(uint32_t MaxContacts, uint32_t Iterations)
    : m_Contacts(MaxContacts), m_ContactResolver(Iterations > 0 ? Iterations : 2 * MaxContacts)
{
}

void Physics::ParticleWorld::StartFrame() {
    for (Particle* Particle : m_Particles)
    {
        Particle->ClearAccumulator();
    }
}

void Physics::ParticleWorld::Integrate(Physics::real DeltaSeconds)
{
    for (Particle* Particle : m_Particles)
    {
        Particle->Integrate(DeltaSeconds);
    }
}

uint32_t Physics::ParticleWorld::GenerateContacts()
{
    ParticleContact* NextContact = m_Contacts.data();
    size_t Limit = m_Contacts.size();
    size_t ContactPosition = 0;
    for (ParticleContactGenerator* Generator : m_ContactGenerators)
    {
        const uint32_t ContactCount = Generator->AddContact({NextContact + ContactPosition, Limit});
        ContactPosition += ContactCount;
        Limit -= ContactCount;
        if (Limit == 0)
        {
            break;
        }
    }

    return static_cast<uint32_t>(ContactPosition);
}

void Physics::ParticleWorld::Run(Physics::real DeltaSeconds)
{
    m_ForceRegistry.UpdateForces(DeltaSeconds);

    Integrate(DeltaSeconds);

    GenerateContacts();

    m_ContactResolver.ResolveContacts(m_Contacts, DeltaSeconds);
}
