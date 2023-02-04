#pragma once

#include "physics/base.h"

#include "math/vector3.h"

namespace physics
{

class Particle;

// Represents a contact between two particles. If second particle is not specified then it is
// assumed that the contact is with the immovable object. In this case the contact normal needs to
// be provided.
class ParticleContact
{
public:
    // Creates a new contact with the given particles and restitution coefficient. The contact
    // normal will be directed from other particle to the main particle and is calculated based on
    // their positions.
    // @param MainParticle - The main particle in the contact. Can't be nullptr.
    // @param OtherParticle - The other particle in the contact. Can't be nullptr.
    // @param Restitution - The coefficient of restitution of the contact. Limited to range [0, 1].
    // @param Penetration - The penetration of the contact. If value is zero then the particles are
    //                      touching. Sign should match the direction of the contact normal.
    // @return - The new contact or nullptr in case of an error.
    static ParticleContact* Create(Particle* MainParticle,
                                   Particle* OtherParticle,
                                   real Restitution,
                                   real Penetration);

    // Creates a new contact between the MainParticle and the immovable object.
    // @param MainParticle - The main particle in the contact. Can't be nullptr.
    // @param Restitution - The coefficient of restitution of the contact. Limited to range [0, 1].
    // @param Penetration - The penetration of the contact. If value is zero then the particles are
    //                      touching. Sign should match the direction of the contact normal.
    // @param ContactNormal - The contact normal. Can't be zero vector.
    static ParticleContact* Create(Particle* MainParticle,
                                   real Restitution,
                                   real Penetration,
                                   const math::Vector3& ContactNormal);

    ~ParticleContact() = default;
    ParticleContact(const ParticleContact&) = default;
    ParticleContact(ParticleContact&&) = default;
    ParticleContact& operator=(const ParticleContact&) = default;
    ParticleContact& operator=(ParticleContact&&) = default;

    [[nodiscard]] Particle* GetMainParticle() const { return m_MainParticle; }
    [[nodiscard]] Particle* GetOtherParticle() const { return m_OtherParticle; }
    [[nodiscard]] real GetRestitution() const { return m_Restitution; }
    [[nodiscard]] real GetPenetration() const { return m_Penetration; }
    [[nodiscard]] const math::Vector3& GetContactNormal() const { return m_ContactNormal; }

    // Resolves the contact. This will calculate the new velocities of the particles and move them
    // to the correct positions so that they are no longer interpenetrating.
    // @param DeltaSeconds - The time step.
    void Resolve(real DeltaSeconds);

protected:
    ParticleContact() = default;

    void ResolveVelocity(real DeltaSeconds);
    void ResolveInterpenetration(real DeltaSeconds);

    [[nodiscard]] real CalculateSeparatingVelocity() const;

    Particle* m_MainParticle = nullptr;
    Particle* m_OtherParticle = nullptr;
    real m_Restitution = PHYSICS_REALC(1.0);
    real m_Penetration = PHYSICS_REALC(0.0);
    math::Vector3 m_ContactNormal;
};

}  // namespace physics