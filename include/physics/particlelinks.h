#pragma once

#include "physics/particlecontacts.h"

namespace physics
{

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
    virtual uint32_t AddContact(Span<ParticleContact> Contacts, uint32_t Limit) = 0;

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

    [[nodiscard]] uint32_t AddContact(Span<ParticleContact> Contacts, uint32_t Limit) override;

protected:
    real m_MaxLength = PHYSICS_REALC(0.0);
    real m_Restitution = PHYSICS_REALC(0.0);
};

// A link that generates a contact if its length changes.
class ParticleRod : public ParticleLink
{
public:
    ParticleRod(Particle* FirstParticle, Particle* SecondParticle, real Length);

    void SetLength(real Length) { m_Length = Length; }
    [[nodiscard]] real GetLength() const { return m_Length; }

    [[nodiscard]] uint32_t AddContact(Span<ParticleContact> Contacts, uint32_t Limit) override;

private:
    real m_Length = PHYSICS_REALC(0.0);
};

}  // namespace physics
