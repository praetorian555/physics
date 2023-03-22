#pragma once

#include "math/point3.h"

#include "physics/containers.h"

namespace physics
{

class Particle;

// Interface used to generate forces and apply them to particles.
class ParticleForceGenerator
{
public:
    virtual ~ParticleForceGenerator() = default;

    // Calculates and applies the force to the given particle.
    virtual void UpdateForce(Particle& Particle, real DeltaSeconds) = 0;
};

// Used to track which force generators are applied to which particles.
class ParticleForceRegistry
{
protected:
    struct Entry
    {
        Particle* Particle;
        ParticleForceGenerator* ForceGenerator;
    };

public:
    void Add(Particle* Particle, ParticleForceGenerator* ForceGenerator);
    void Remove(Particle* Particle, ParticleForceGenerator* ForceGenerator);
    void Clear();

    void UpdateForces(real DeltaSeconds);

private:
    Array<Entry> m_Entries;
};

// Applies a gravitational force to the particle. Can be used on multiple particles.
class ParticleGravity : public ParticleForceGenerator
{
public:
    explicit ParticleGravity(const math::Vector3& Gravity) : m_Gravity(Gravity) {}

    void UpdateForce(Particle& Particle, real DeltaSeconds) override;

    void SetGravity(const math::Vector3& Gravity) { m_Gravity = Gravity; }
    [[nodiscard]] const math::Vector3& GetGravity() const { return m_Gravity; }

private:
    math::Vector3 m_Gravity;
};

// Applies a drag force to the particle. Can be used on multiple particles.
class ParticleDrag : public ParticleForceGenerator
{
public:
    // K1 coefficient is applied to the speed of the particle. K2 is applied to the square of the
    // speed.
    ParticleDrag(real K1, real K2) : m_K1(K1), m_K2(K2) {}

    void UpdateForce(Particle& Particle, real DeltaSeconds) override;

    void SetK1(real K1) { m_K1 = K1; }
    void SetK2(real K2) { m_K2 = K2; }
    [[nodiscard]] real GetK1() const { return m_K1; }
    [[nodiscard]] real GetK2() const { return m_K2; }

private:
    real m_K1;
    real m_K2;
};

// Applies a spring force to the particle. Other side of the spring is also connected to a particle.
// Can be used on multiple particles.
class ParticleSpring : public ParticleForceGenerator
{
public:
    ParticleSpring(Particle* Other, real SpringConstant, real RestLength)
        : m_Other(Other), m_SpringConstant(SpringConstant), m_RestLength(RestLength)
    {
    }

    void UpdateForce(Particle& Particle, real DeltaSeconds) override;

    void SetOther(Particle* Other) { m_Other = Other; }
    [[nodiscard]] Particle* GetOther() const { return m_Other; }
    void SetSpringConstant(real SpringConstant) { m_SpringConstant = SpringConstant; }
    [[nodiscard]] real GetSpringConstant() const { return m_SpringConstant; }
    void SetRestLength(real RestLength) { m_RestLength = RestLength; }
    [[nodiscard]] real GetRestLength() const { return m_RestLength; }

private:
    Particle* m_Other;
    real m_SpringConstant;
    real m_RestLength;
};

// Applies a spring force to the particle. Other side of the spring is anchored to a point. Can be
// applied to multiple particles.
class ParticleAnchoredSpring : public ParticleForceGenerator
{
public:
    ParticleAnchoredSpring(const math::Point3& Anchor, real SpringConstant, real RestLength)
        : m_Anchor(Anchor), m_SpringConstant(SpringConstant), m_RestLength(RestLength)
    {
    }

    void UpdateForce(Particle& Particle, real DeltaSeconds) override;

    void SetAnchor(const math::Point3& Anchor) { m_Anchor = Anchor; }
    [[nodiscard]] const math::Point3& GetAnchor() const { return m_Anchor; }
    void SetSpringConstant(real SpringConstant) { m_SpringConstant = SpringConstant; }
    [[nodiscard]] real GetSpringConstant() const { return m_SpringConstant; }
    void SetRestLength(real RestLength) { m_RestLength = RestLength; }
    [[nodiscard]] real GetRestLength() const { return m_RestLength; }

private:
    math::Point3 m_Anchor;
    real m_SpringConstant;
    real m_RestLength;
};

// Applies a spring force to the particle only if the spring is longer then the resting length.
// Other side of the spring is also connected to a particle. Can be used on multiple particles.
class ParticleBungee : public ParticleForceGenerator
{
public:
    ParticleBungee(Particle* Other, real SpringConstant, real RestLength)
        : m_Other(Other), m_SpringConstant(SpringConstant), m_RestLength(RestLength)
    {
    }

    void UpdateForce(Particle& Particle, real DeltaSeconds) override;

    void SetOther(Particle* Other) { m_Other = Other; }
    [[nodiscard]] Particle* GetOther() const { return m_Other; }
    void SetSpringConstant(real SpringConstant) { m_SpringConstant = SpringConstant; }
    [[nodiscard]] real GetSpringConstant() const { return m_SpringConstant; }
    void SetRestLength(real RestLength) { m_RestLength = RestLength; }
    [[nodiscard]] real GetRestLength() const { return m_RestLength; }

private:
    Particle* m_Other;
    real m_SpringConstant;
    real m_RestLength;
};

// Applies a buoyancy force to the particle if it is submerged in water. Assumes that water plane is
// parallel to XZ plane.
class ParticleBuoyancy : public ParticleForceGenerator
{
public:
    // @brief Creates a new ParticleBuoyancy force generator.
    // @param MaxDepth The maximum depth of the object before it generates maximum buoyancy force.
    // @param Volume The volume of the object.
    // @param WaterHeight The height of the water plane above Y=0. The plane will be parallel to the
    // XZ plane.
    // @param LiquidDensity The density of the liquid.
    ParticleBuoyancy(real MaxDepth, real Volume, real WaterHeight, real LiquidDensity)
        : m_MaxDepth(MaxDepth),
          m_Volume(Volume),
          m_WaterHeight(WaterHeight),
          m_LiquidDensity(LiquidDensity)
    {
    }

    void UpdateForce(Particle& Particle, real DeltaSeconds) override;

    void SetMaxDepth(real MaxDepth) { m_MaxDepth = MaxDepth; }
    [[nodiscard]] real GetMaxDepth() const { return m_MaxDepth; }
    void SetVolume(real Volume) { m_Volume = Volume; }
    [[nodiscard]] real GetVolume() const { return m_Volume; }
    void SetWaterHeight(real WaterHeight) { m_WaterHeight = WaterHeight; }
    [[nodiscard]] real GetWaterHeight() const { return m_WaterHeight; }
    void SetLiquidDensity(real LiquidDensity) { m_LiquidDensity = LiquidDensity; }
    [[nodiscard]] real GetLiquidDensity() const { return m_LiquidDensity; }

private:
    real m_MaxDepth;
    real m_Volume;
    real m_WaterHeight;
    real m_LiquidDensity;
};
}  // namespace physics
