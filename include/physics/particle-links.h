#pragma once

#include "physics/particle-contacts.h"

namespace Physics
{

/**
 * Base class used to link to particles together. It generates contact when the constraint of the
 * link is violated. Used as a base class for the ParticleCable and ParticleRod classes. It can also
 * be used for springs that have extension limit.
 */
class ParticleLink
{
public:
    ParticleLink(Particle* first_particle, Particle* second_particle);
    virtual ~ParticleLink() = default;

    /**
     * Fills the given contact structure with the contact needed to keep the link from being
     * violated.
     * @param Contacts - The contact array to fill.
     * @return - Returns 1 in case of a success, 0 otherwise.
     */
    virtual uint32_t AddContact(Span<ParticleContact> contacts) = 0;

    void SetFirstParticle(Particle* Particle) { m_first_particle = Particle; }
    void SetSecondParticle(Particle* Particle) { m_second_particle = Particle; }
    [[nodiscard]] Particle* GetFirstParticle() const { return m_first_particle; }
    [[nodiscard]] Particle* GetSecondParticle() const { return m_second_particle; }

protected:
    /** Returns the current length of the link. */
    [[nodiscard]] virtual real GetCurrentLength() const;

    Particle* m_first_particle = nullptr;
    Particle* m_second_particle = nullptr;
};

/** A link that generates a contact if it is overextended. */
class ParticleCable : public ParticleLink
{
public:
    ParticleCable(Particle* first_particle,
                  Particle* second_particle,
                  real max_length,
                  real restitution);

    void SetMaxLength(real max_length) { m_max_length = max_length; }
    void SetRestitution(real restitution) { m_restitution = restitution; }
    [[nodiscard]] real GetMaxLength() const { return m_max_length; }
    [[nodiscard]] real GetRestitution() const { return m_restitution; }

    [[nodiscard]] uint32_t AddContact(Span<ParticleContact> contacts) override;

protected:
    real m_max_length = PHYSICS_REALC(0.0);
    real m_restitution = PHYSICS_REALC(0.0);
};

/** A link that generates a contact if its length changes. */
class ParticleRod : public ParticleLink
{
public:
    ParticleRod(Particle* first_particle, Particle* second_particle, real length);

    void SetLength(real length) { m_length = length; }
    [[nodiscard]] real GetLength() const { return m_length; }

    [[nodiscard]] uint32_t AddContact(Span<ParticleContact> contacts) override;

private:
    real m_length = PHYSICS_REALC(0.0);
};

}  // namespace Physics
