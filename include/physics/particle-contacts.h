#pragma once

#include "physics/containers.h"

namespace physics
{

class Particle;

/**
 * Represents a contact between two particles. If second particle is not specified then it is
 * assumed that the contact is with the immovable object. In this case the contact normal needs to
 * be provided.
 */
class ParticleContact
{
public:
    /**
     * Creates a new contact with the given particles and restitution coefficient. The contact
     * normal will be directed from other particle to the main particle and is calculated based on
     * their positions.
     * @param main_particle - The main particle in the contact. Can't be nullptr.
     * @param other_particle - The other particle in the contact. Can't be nullptr.
     * @param restitution - The coefficient of restitution of the contact. Limited to range [0, 1].
     * @param penetration - The penetration of the contact. If value is zero then the particles are
     *                      touching. Sign should match the direction of the contact normal.
     */
    ParticleContact(Particle* main_particle,
                    Particle* other_particle,
                    real restitution,
                    real penetration);

    /**
     * Creates a new contact between the MainParticle and the immovable object.
     * @param main_particle - The main particle in the contact. Can't be nullptr.
     * @param restitution - The coefficient of restitution of the contact. Limited to range [0, 1].
     * @param penetration - The penetration of the contact. If value is zero then the particles are
     *                      touching. Sign should match the direction of the contact normal.
     * @param contact_normal - The contact normal. Can't be zero vector.
     */
    ParticleContact(Particle* main_particle,
                    real restitution,
                    real penetration,
                    const math::Vector3& contact_normal);

    ParticleContact(Particle* main_particle,
                    Particle* other_particle,
                    real restitution,
                    real penetration,
                    const math::Vector3& contact_normal);

    ParticleContact() = default;
    ~ParticleContact() = default;
    ParticleContact(const ParticleContact&) = default;
    ParticleContact(ParticleContact&&) = default;
    ParticleContact& operator=(const ParticleContact&) = default;
    ParticleContact& operator=(ParticleContact&&) = default;

    /**
     * Checks if the contact is valid. A contact is valid: if the main particle is not nullptr,
     * normal is not a zero vector, and restitution is in the range [0, 1]. Also, no real values are
     * NANs.
     * @return True if the contact is valid, false otherwise.
     */
    [[nodiscard]] bool IsValid() const;

    [[nodiscard]] Particle* GetMainParticle() const { return m_main_particle; }
    [[nodiscard]] Particle* GetOtherParticle() const { return m_other_particle; }
    [[nodiscard]] real GetRestitution() const { return m_restitution; }
    [[nodiscard]] real GetPenetration() const { return m_penetration; }
    [[nodiscard]] const math::Vector3& GetContactNormal() const { return m_contact_normal; }

    /**
     * Resolves the contact. This will calculate the new velocities of the particles and move them
     * to the correct positions so that they are no longer interpenetrating.
     * @param delta_seconds - The time step.
     */
    void Resolve(real delta_seconds);

    [[nodiscard]] real CalculateSeparatingVelocity() const;

protected:
    void ResolveVelocity(real delta_seconds);
    void ResolveInterpenetration(real delta_seconds);

    Particle* m_main_particle = nullptr;
    Particle* m_other_particle = nullptr;
    real m_restitution = PHYSICS_REALC(0.0);
    real m_penetration = PHYSICS_REALC(0.0);
    math::Vector3 m_contact_normal = math::Vector3::Zero;
};

/** Used to resolve a set of particle contacts for both penetration and velocity. */
class ParticleContactResolver
{
public:
    explicit ParticleContactResolver(uint32_t iterations) : m_iterations(iterations) {}

    void SetIterations(uint32_t iterations) { m_iterations = iterations; }
    [[nodiscard]] uint32_t GetIterations() const { return m_iterations; }

    [[nodiscard]] uint32_t GetIterationsUsed() const { return m_iterations_used; }

    /**
     * Resolves a set of particle contacts for both penetration and velocity.
     * @param contacts - The contacts to resolve.
     * @param delta_seconds - The time step.
     */
    void ResolveContacts(Span<ParticleContact> contacts, real delta_seconds);

private:
    uint32_t m_iterations = 0;
    uint32_t m_iterations_used = 0;
};

/** Base class used to generate contacts. */
class ParticleContactGenerator
{
public:
    virtual ~ParticleContactGenerator() = default;

    /**
     * Fills the given contact array with the generated contacts. The limit parameter defines the
     * maximum number of contacts that can be written to the array. The method returns the number
     * of contacts that have been written.
     * @param contacts - The array of contacts to fill.
     * @return - The number of contacts that have been written.
     */
    virtual uint32_t AddContact(Span<ParticleContact> contacts) = 0;
};

}  // namespace physics
