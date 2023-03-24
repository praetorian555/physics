#include "physics/force-generator.h"

#include "physics/body.h"

void Physics::ForceRegistry::Add(Physics::RigidBody* body, Physics::ForceGenerator* force_generator)
{
    PHYSICS_ASSERT(body != nullptr);
    PHYSICS_ASSERT(force_generator != nullptr);

    m_entries.push_back({body, force_generator});
}

void Physics::ForceRegistry::Remove(Physics::RigidBody* body,
                                    Physics::ForceGenerator* force_generator)
{
    std::erase_if(m_entries, [body, force_generator](const Entry& entry)
                  { return entry.body == body && entry.force_generator == force_generator; });
}

void Physics::ForceRegistry::Clear()
{
    m_entries.clear();
}

void Physics::ForceRegistry::UpdateForces(Physics::real delta_seconds)
{
    for (const Entry& entry : m_entries)
    {
        entry.force_generator->UpdateForce(*entry.body, delta_seconds);
    }
}

void Physics::Gravity::UpdateForce(RigidBody& body, real delta_seconds)
{
    PHYSICS_UNUSED(delta_seconds);
    if (!body.HasFiniteMass())
    {
        return;
    }
    body.AddForce(m_gravity * body.GetMass());
}

Physics::Spring::Spring(const math::Point3& connection_point_local,
                        Physics::RigidBody* other_body,
                        const math::Point3& other_connection_point_local,
                        Physics::real spring_constant,
                        Physics::real rest_length)
    : m_connection_point_local(connection_point_local),
      m_other_body(other_body),
      m_other_connection_point_local(other_connection_point_local),
      m_spring_constant(spring_constant),
      m_rest_length(rest_length)
{
}

void Physics::Spring::SetConnectionPointLocal(const math::Point3& connection_point_local)
{
    m_connection_point_local = connection_point_local;
}

const math::Point3& Physics::Spring::GetConnectionPointLocal() const
{
    return m_connection_point_local;
}

void Physics::Spring::SetOtherBody(Physics::RigidBody* other_body)
{
    m_other_body = other_body;
}

Physics::RigidBody* Physics::Spring::GetOtherBody() const
{
    return m_other_body;
}

void Physics::Spring::SetOtherConnectionPointLocal(const math::Point3& other_connection_point_local)
{
    m_other_connection_point_local = other_connection_point_local;
}

const math::Point3& Physics::Spring::GetOtherConnectionPointLocal() const
{
    return m_other_connection_point_local;
}

void Physics::Spring::SetSpringConstant(Physics::real spring_constant)
{
    m_spring_constant = spring_constant;
}

Physics::real Physics::Spring::GetSpringConstant() const
{
    return m_spring_constant;
}

void Physics::Spring::SetRestLength(Physics::real rest_length)
{
    m_rest_length = rest_length;
}

Physics::real Physics::Spring::GetRestLength() const
{
    return m_rest_length;
}

void Physics::Spring::UpdateForce(Physics::RigidBody& body, Physics::real delta_seconds)
{
    PHYSICS_UNUSED(delta_seconds);

    const math::Point3 connection_point_world = body.GetTransform()(m_connection_point_local);
    const math::Point3 other_connection_point_world =
        m_other_body->GetTransform()(m_other_connection_point_local);

    math::Vector3 length_vector = connection_point_world - other_connection_point_world;
    const real length = length_vector.Length();
    const real spring_force = m_spring_constant * (length - m_rest_length);
    length_vector = math::Normalize(length_vector);

    body.AddForce(-length_vector * spring_force);
}
