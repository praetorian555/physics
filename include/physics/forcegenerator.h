#pragma once

#include "math/point3.h"

#include "physics/base.h"
#include "physics/containers.h"

namespace physics
{

class RigidBody;

/**
 * Represents a force generator interface used to generate forces and apply them to rigid bodies.
 */
class ForceGenerator
{
public:
    virtual ~ForceGenerator() = default;

    /**
     * Calculates and applies the force to the given rigid body.
     * @param Body Rigid body to apply the force to.
     * @param DeltaSeconds Time since last frame.
     */
    virtual void UpdateForce(RigidBody& Body, real DeltaSeconds) = 0;
};

/**
 * Used to track which force generators are applied to which rigid bodies.
 */
class ForceRegistry
{
protected:
    struct Entry
    {
        RigidBody* Body;
        ForceGenerator* ForceGenerator;
    };

public:
    void Add(RigidBody* Body, ForceGenerator* ForceGenerator);
    void Remove(RigidBody* Body, ForceGenerator* ForceGenerator);
    void Clear();

    void UpdateForces(real DeltaSeconds);

protected:
    Array<Entry> m_Entries;
};

/**
 * Represents a force generator that applies gravity force to the rigid body. Can be used on
 * multiple rigid bodies.
 */
class Gravity : public ForceGenerator
{
public:
    explicit Gravity(const math::Vector3& Gravity) : m_Gravity(Gravity) {}

    void UpdateForce(RigidBody& Body, real DeltaSeconds) override;

    void SetGravity(const math::Vector3& Gravity) { m_Gravity = Gravity; }
    [[nodiscard]] const math::Vector3& GetGravity() const { return m_Gravity; }

private:
    math::Vector3 m_Gravity;
};

/**
 * Represents a force generator that applies a spring force on the rigid body.
 */
class Spring : public ForceGenerator
{
public:
    /**
     * Creates a new spring force generator.
     * @param ConnectionPointLocal Connection point of the spring on the rigid body. This is expressed
     * in the local space of the rigid body that will be passed in UpdateForce.
     * @param OtherBody Rigid body attached to the other side of the spring.
     * @param OtherConnectionPointLocal Connection point of the spring on the other rigid body. This
     * is expressed in the local space of the other rigid body.
     * @param SpringConstant Spring constant.
     * @param RestLength Rest length of the spring.
     */
    Spring(const math::Point3& ConnectionPointLocal,
           RigidBody* OtherBody,
           const math::Point3& OtherConnectionPointLocal,
           real SpringConstant,
           real RestLength);

    void SetConnectionPointLocal(const math::Point3& ConnectionPointLocal);
    [[nodiscard]] const math::Point3& GetConnectionPointLocal() const;

    void SetOtherBody(RigidBody* OtherBody);
    [[nodiscard]] RigidBody* GetOtherBody() const;

    void SetOtherConnectionPointLocal(const math::Point3& OtherConnectionPointLocal);
    [[nodiscard]] const math::Point3& GetOtherConnectionPointLocal() const;

    void SetSpringConstant(real SpringConstant);
    [[nodiscard]] real GetSpringConstant() const;

    void SetRestLength(real RestLength);
    [[nodiscard]] real GetRestLength() const;

    void UpdateForce(RigidBody& Body, real DeltaSeconds) override;

private:
    /**
     * Point where one rigid body is connected to the spring. This is expressed in the local space
     * of that rigid body. Since this is a rigid body that will passed as an argument to the
     * UpdateForce method, if we want to use this generator for different rigid bodies, we need to
     * change this value for each one.
     */
    math::Point3 m_ConnectionPointLocal;

    /**
     * Rigid body attached to the other side of the spring.
     */
    RigidBody* m_OtherBody;

    /**
     * Point where the other rigid body is connected to the spring. This is expressed in the local
     * space of that rigid body.
     */
    math::Point3 m_OtherConnectionPointLocal;

    real m_SpringConstant;
    real m_RestLength;
};

}  // namespace physics
