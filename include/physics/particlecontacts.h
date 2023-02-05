#pragma once

#include <span>

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
    ParticleContact(Particle* MainParticle,
                    Particle* OtherParticle,
                    real Restitution,
                    real Penetration);

    // Creates a new contact between the MainParticle and the immovable object.
    // @param MainParticle - The main particle in the contact. Can't be nullptr.
    // @param Restitution - The coefficient of restitution of the contact. Limited to range [0, 1].
    // @param Penetration - The penetration of the contact. If value is zero then the particles are
    //                      touching. Sign should match the direction of the contact normal.
    // @param ContactNormal - The contact normal. Can't be zero vector.
    ParticleContact(Particle* MainParticle,
                    real Restitution,
                    real Penetration,
                    const math::Vector3& ContactNormal);

    ParticleContact() = default;
    ~ParticleContact() = default;
    ParticleContact(const ParticleContact&) = default;
    ParticleContact(ParticleContact&&) = default;
    ParticleContact& operator=(const ParticleContact&) = default;
    ParticleContact& operator=(ParticleContact&&) = default;

    // Checks if the contact is valid. A contact is valid: if the main particle is not nullptr,
    // normal is not a zero vector, and restitution is in the range [0, 1]. Also, no real values are
    // NANs.
    // @return True if the contact is valid, false otherwise.
    [[nodiscard]] bool IsValid() const;

    [[nodiscard]] Particle* GetMainParticle() const { return m_MainParticle; }
    [[nodiscard]] Particle* GetOtherParticle() const { return m_OtherParticle; }
    [[nodiscard]] real GetRestitution() const { return m_Restitution; }
    [[nodiscard]] real GetPenetration() const { return m_Penetration; }
    [[nodiscard]] const math::Vector3& GetContactNormal() const { return m_ContactNormal; }

    // Resolves the contact. This will calculate the new velocities of the particles and move them
    // to the correct positions so that they are no longer interpenetrating.
    // @param DeltaSeconds - The time step.
    void Resolve(real DeltaSeconds);

    [[nodiscard]] real CalculateSeparatingVelocity() const;

protected:
    void ResolveVelocity(real DeltaSeconds);
    void ResolveInterpenetration(real DeltaSeconds);

    Particle* m_MainParticle = nullptr;
    Particle* m_OtherParticle = nullptr;
    real m_Restitution = PHYSICS_REALC(0.0);
    real m_Penetration = PHYSICS_REALC(0.0);
    math::Vector3 m_ContactNormal = math::Vector3::Zero;
};

// Used to resolve a set of particle contacts for both penetration and velocity.
class ParticleContactResolver
{
public:
    explicit ParticleContactResolver(uint32_t Iterations) : m_Iterations(Iterations) {}

    void SetIterations(uint32_t Iterations) { m_Iterations = Iterations; }
    [[nodiscard]] uint32_t GetIterations() const { return m_Iterations; }

    [[nodiscard]] uint32_t GetIterationsUsed() const { return m_IterationsUsed; }

    // Resolves a set of particle contacts for both penetration and velocity.
    // @param Contacts - The contacts to resolve.
    // @param DeltaSeconds - The time step.
    void ResolveContacts(std::span<ParticleContact> Contacts, real DeltaSeconds);

private:
    uint32_t m_Iterations = 0;
    uint32_t m_IterationsUsed = 0;
};

// Base class used to generate contacts.
class ParticleContactGenerator
{
public:
    virtual ~ParticleContactGenerator() = default;

    // Fills the given contact array with the generated contacts. The limit parameter defines the
    // maximum number of contacts that can be written to the array. The method returns the number
    // of contacts that have been written.
    // @param Contacts - The array of contacts to fill.
    // @param Limit - The maximum number of contacts to write.
    // @return - The number of contacts that have been written.
    virtual uint32_t AddContact(std::span<ParticleContact> Contacts, uint32_t Limit) = 0;
};

// Base class used to link to particles together. It generates contact when the constraint of the
// link is violated. Used as a base class for the ParticleCable and ParticleRod classes. It can also
// be used for springs that have extension limit.
class ParticleLink
{
public:
    ParticleLink(Particle* FirstParticle, Particle* SecondParticle);
    virtual ~ParticleLink() = default;

    // Fills the given contact structure with the contact needed to keep the link from being
    // violated.
    // @param Contact - The contact to fill.
    // @param Limit - The maximum number of contacts to write.
    // @return - Returns 1 in case of a success, 0 otherwise.
    virtual uint32_t AddContact(std::span<ParticleContact> Contacts, uint32_t Limit) = 0;

    void SetFirstParticle(Particle* Particle) { m_FirstParticle = Particle; }
    void SetSecondParticle(Particle* Particle) { m_SecondParticle = Particle; }
    [[nodiscard]] Particle* GetFirstParticle() const { return m_FirstParticle; }
    [[nodiscard]] Particle* GetSecondParticle() const { return m_SecondParticle; }

protected:
    // Returns the current length of the link.
    [[nodiscard]] virtual real GetCurrentLength() const;

    Particle* m_FirstParticle = nullptr;
    Particle* m_SecondParticle = nullptr;
};

// A link that generates a contact if it is overextended.
class ParticleCable : public ParticleLink
{
public:
    ParticleCable(Particle* FirstParticle,
                  Particle* SecondParticle,
                  real MaxLength,
                  real Restitution);

    void SetMaxLength(real MaxLength) { m_MaxLength = MaxLength; }
    void SetRestitution(real Restitution) { m_Restitution = Restitution; }
    [[nodiscard]] real GetMaxLength() const { return m_MaxLength; }
    [[nodiscard]] real GetRestitution() const { return m_Restitution; }

    [[nodiscard]] uint32_t AddContact(std::span<ParticleContact> Contacts, uint32_t Limit) override;

protected:
    real m_MaxLength = PHYSICS_REALC(0.0);
    real m_Restitution = PHYSICS_REALC(0.0);
};

}  // namespace physics
