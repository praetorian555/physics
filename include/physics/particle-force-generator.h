#pragma once

#include "physics/containers.h"

namespace physics
{

class Particle;

/** Interface used to generate forces and apply them to particles. */
class ParticleForceGenerator
{
public:
    virtual ~ParticleForceGenerator() = default;

    /**
     * Applies force to the given particle.
     * @param particle Particle to apply the force to.
     * @param delta_seconds Time step.
     */
    virtual void UpdateForce(Particle& particle, real delta_seconds) = 0;
};

/** Used to track which force generators are applied to which particles. */
class ParticleForceRegistry
{
protected:
    struct Entry
    {
        Particle* particle;
        ParticleForceGenerator* force_generator;
    };

public:
    void Add(Particle* particle, ParticleForceGenerator* force_generator);
    void Remove(Particle* particle, ParticleForceGenerator* force_generator);
    void Clear();

    void UpdateForces(real delta_seconds);

private:
    Array<Entry> m_entries;
};

/** Applies a gravitational force to the particle. Can be used on multiple particles. */
class ParticleGravity : public ParticleForceGenerator
{
public:
    explicit ParticleGravity(const math::Vector3& gravity) : m_gravity(gravity) {}

    void UpdateForce(Particle& particle, real delta_seconds) override;

    void SetGravity(const math::Vector3& gravity) { m_gravity = gravity; }
    [[nodiscard]] const math::Vector3& GetGravity() const { return m_gravity; }

private:
    math::Vector3 m_gravity;
};

/** Applies a drag force to the particle. Can be used on multiple particles. */
class ParticleDrag : public ParticleForceGenerator
{
public:
    // K1 coefficient is applied to the speed of the particle. K2 is applied to the square of the
    // speed.
    ParticleDrag(real k1, real k2) : m_k1(k1), m_k2(k2) {}

    void UpdateForce(Particle& particle, real delta_seconds) override;

    void SetK1(real k1) { m_k1 = k1; }
    void SetK2(real k2) { m_k2 = k2; }
    [[nodiscard]] real GetK1() const { return m_k1; }
    [[nodiscard]] real GetK2() const { return m_k2; }

private:
    real m_k1;
    real m_k2;
};

/**
 * Applies a spring force to the particle. Other side of the spring is also connected to a particle.
 * Can be used on multiple particles.
 */
class ParticleSpring : public ParticleForceGenerator
{
public:
    ParticleSpring(Particle* other, real spring_constant, real rest_length)
        : m_other(other), m_spring_constant(spring_constant), m_rest_length(rest_length)
    {
    }

    void UpdateForce(Particle& particle, real delta_seconds) override;

    void SetOther(Particle* other) { m_other = other; }
    [[nodiscard]] Particle* GetOther() const { return m_other; }
    void SetSpringConstant(real spring_constant) { m_spring_constant = spring_constant; }
    [[nodiscard]] real GetSpringConstant() const { return m_spring_constant; }
    void SetRestLength(real rest_length) { m_rest_length = rest_length; }
    [[nodiscard]] real GetRestLength() const { return m_rest_length; }

private:
    Particle* m_other;
    real m_spring_constant;
    real m_rest_length;
};

/**
 * Applies a spring force to the particle. Other side of the spring is anchored to a point. Can be
 * applied to multiple particles.
 */
class ParticleAnchoredSpring : public ParticleForceGenerator
{
public:
    ParticleAnchoredSpring(const math::Point3& Anchor, real SpringConstant, real RestLength)
        : m_anchor(Anchor), m_spring_constant(SpringConstant), m_rest_length(RestLength)
    {
    }

    void UpdateForce(Particle& particle, real delta_seconds) override;

    void SetAnchor(const math::Point3& anchor) { m_anchor = anchor; }
    [[nodiscard]] const math::Point3& GetAnchor() const { return m_anchor; }
    void SetSpringConstant(real spring_constant) { m_spring_constant = spring_constant; }
    [[nodiscard]] real GetSpringConstant() const { return m_spring_constant; }
    void SetRestLength(real rest_length) { m_rest_length = rest_length; }
    [[nodiscard]] real GetRestLength() const { return m_rest_length; }

private:
    math::Point3 m_anchor;
    real m_spring_constant;
    real m_rest_length;
};

/**
 * Applies a spring force to the particle only if the spring is longer then the resting length.
 * Other side of the spring is also connected to a particle. Can be used on multiple particles.
 */
class ParticleBungee : public ParticleForceGenerator
{
public:
    ParticleBungee(Particle* other, real spring_constant, real rest_length)
        : m_other(other), m_spring_constant(spring_constant), m_rest_length(rest_length)
    {
    }

    void UpdateForce(Particle& particle, real delta_seconds) override;

    void SetOther(Particle* other) { m_other = other; }
    [[nodiscard]] Particle* GetOther() const { return m_other; }
    void SetSpringConstant(real spring_constant) { m_spring_constant = spring_constant; }
    [[nodiscard]] real GetSpringConstant() const { return m_spring_constant; }
    void SetRestLength(real rest_length) { m_rest_length = rest_length; }
    [[nodiscard]] real GetRestLength() const { return m_rest_length; }

private:
    Particle* m_other;
    real m_spring_constant;
    real m_rest_length;
};

/**
 * Applies a buoyancy force to the particle if it is submerged in water. Assumes that water plane is
 * parallel to XZ plane.
 */
class ParticleBuoyancy : public ParticleForceGenerator
{
public:
    /**
     * @brief Creates a new ParticleBuoyancy force generator.
     * @param max_depth The maximum depth of the object before it generates maximum buoyancy force.
     * @param volume The volume of the object.
     * @param water_height The height of the water plane above Y=0. The plane will be parallel to
     * the XZ plane.
     * @param liquid_density The density of the liquid.
     */
    ParticleBuoyancy(real max_depth, real volume, real water_height, real liquid_density)
        : m_max_depth(max_depth),
          m_volume(volume),
          m_water_height(water_height),
          m_liquid_density(liquid_density)
    {
    }

    void UpdateForce(Particle& particle, real delta_seconds) override;

    void SetMaxDepth(real max_depth) { m_max_depth = max_depth; }
    [[nodiscard]] real GetMaxDepth() const { return m_max_depth; }
    void SetVolume(real volume) { m_volume = volume; }
    [[nodiscard]] real GetVolume() const { return m_volume; }
    void SetWaterHeight(real water_height) { m_water_height = water_height; }
    [[nodiscard]] real GetWaterHeight() const { return m_water_height; }
    void SetLiquidDensity(real liquid_density) { m_liquid_density = liquid_density; }
    [[nodiscard]] real GetLiquidDensity() const { return m_liquid_density; }

private:
    real m_max_depth;
    real m_volume;
    real m_water_height;
    real m_liquid_density;
};

}  // namespace physics
