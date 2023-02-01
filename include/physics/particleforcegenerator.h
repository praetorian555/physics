#pragma once

#include <vector>

#include "math/vector3.h"

#include "physics/base.h"

namespace physics
{

class Particle;

// Interface used to generate forces and apply them to particles.
class ParticleForceGenerator
{
public:
    virtual ~ParticleForceGenerator() = default;

    // Calculates and applies the force to the given particle.
    virtual void UpdateForce(Particle& Particle, float DeltaSeconds) = 0;
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

    void UpdateForces(float DeltaSeconds);

private:
    std::vector<Entry> m_Entries;
};

// Applies a gravitational force to the particle. Can be used on multiple particles.
class ParticleGravity : public ParticleForceGenerator
{
public:
    explicit ParticleGravity(const math::Vector3& Gravity) : m_Gravity(Gravity) {}

    void UpdateForce(Particle& Particle, float DeltaSeconds) override;

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

    void UpdateForce(Particle& Particle, float DeltaSeconds) override;

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

    void UpdateForce(Particle& Particle, float DeltaSeconds) override;

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

}  // namespace physics
