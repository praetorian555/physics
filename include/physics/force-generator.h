#pragma once

#include "physics/base.h"
#include "physics/containers.h"

namespace Physics
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
     * @param body Rigid body to apply the force to.
     * @param delta_seconds Time since last frame.
     */
    virtual void UpdateForce(RigidBody& body, real delta_seconds) = 0;
};

/**
 * Used to track which force generators are applied to which rigid bodies.
 */
class ForceRegistry
{
protected:
    struct Entry
    {
        RigidBody* body;
        ForceGenerator* force_generator;
    };

public:
    void Add(RigidBody* body, ForceGenerator* force_generator);
    void Remove(RigidBody* entry, ForceGenerator* force_generator);
    void Clear();

    void UpdateForces(real delta_seconds);

protected:
    Array<Entry> m_entries;
};

/**
 * Represents a force generator that applies gravity force to the rigid body. Can be used on
 * multiple rigid bodies.
 */
class Gravity : public ForceGenerator
{
public:
    explicit Gravity(const math::Vector3& Gravity) : m_gravity(Gravity) {}

    void UpdateForce(RigidBody& body, real delta_seconds) override;

    void SetGravity(const math::Vector3& gravity) { m_gravity = gravity; }
    [[nodiscard]] const math::Vector3& GetGravity() const { return m_gravity; }

private:
    math::Vector3 m_gravity;
};

/**
 * Represents a force generator that applies a spring force on the rigid body.
 */
class Spring : public ForceGenerator
{
public:
    /**
     * Creates a new spring force generator.
     * @param connection_point_local Connection point of the spring on the rigid body. This is expressed
     * in the local space of the rigid body that will be passed in UpdateForce.
     * @param other_body Rigid body attached to the other side of the spring.
     * @param other_connection_point_local Connection point of the spring on the other rigid body. This
     * is expressed in the local space of the other rigid body.
     * @param spring_constant Spring constant.
     * @param rest_length Rest length of the spring.
     */
    Spring(const math::Point3& connection_point_local,
           RigidBody* other_body,
           const math::Point3& other_connection_point_local,
           real spring_constant,
           real rest_length);

    void SetConnectionPointLocal(const math::Point3& connection_point_local);
    [[nodiscard]] const math::Point3& GetConnectionPointLocal() const;

    void SetOtherBody(RigidBody* other_body);
    [[nodiscard]] RigidBody* GetOtherBody() const;

    void SetOtherConnectionPointLocal(const math::Point3& other_connection_point_local);
    [[nodiscard]] const math::Point3& GetOtherConnectionPointLocal() const;

    void SetSpringConstant(real spring_constant);
    [[nodiscard]] real GetSpringConstant() const;

    void SetRestLength(real rest_length);
    [[nodiscard]] real GetRestLength() const;

    void UpdateForce(RigidBody& body, real delta_seconds) override;

private:
    /**
     * Point where one rigid body is connected to the spring. This is expressed in the local space
     * of that rigid body. Since this is a rigid body that will passed as an argument to the
     * UpdateForce method, if we want to use this generator for different rigid bodies, we need to
     * change this value for each one.
     */
    math::Point3 m_connection_point_local;

    /**
     * Rigid body attached to the other side of the spring.
     */
    RigidBody* m_other_body;

    /**
     * Point where the other rigid body is connected to the spring. This is expressed in the local
     * space of that rigid body.
     */
    math::Point3 m_other_connection_point_local;

    real m_spring_constant;
    real m_rest_length;
};

}  // namespace Physics
