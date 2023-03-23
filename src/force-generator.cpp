#include "physics/force-generator.h"

#include "physics/body.h"

void physics::ForceRegistry::Add(physics::RigidBody* body, physics::ForceGenerator* force_generator)
{
    assert(body != nullptr);
    assert(force_generator != nullptr);

    m_entries.push_back({body, force_generator});
}

void physics::ForceRegistry::Remove(physics::RigidBody* body,
                                    physics::ForceGenerator* force_generator)
{
    std::erase_if(m_entries, [body, force_generator](const Entry& entry)
                  { return entry.body == body && entry.force_generator == force_generator; });
}

void physics::ForceRegistry::Clear()
{
    m_entries.clear();
}

void physics::ForceRegistry::UpdateForces(physics::real delta_seconds)
{
    for (const Entry& entry : m_entries)
    {
        entry.force_generator->UpdateForce(*entry.body, delta_seconds);
    }
}

void physics::Gravity::UpdateForce(RigidBody& body, real delta_seconds)
{
    PHYSICS_UNUSED(delta_seconds);
    if (!body.HasFiniteMass())
    {
        return;
    }
    body.AddForce(m_gravity * body.GetMass());
}

physics::Spring::Spring(const math::Point3& connection_point_local,
                        physics::RigidBody* other_body,
                        const math::Point3& other_connection_point_local,
                        physics::real spring_constant,
                        physics::real rest_length)
    : m_connection_point_local(connection_point_local),
      m_other_body(other_body),
      m_other_connection_point_local(other_connection_point_local),
      m_spring_constant(spring_constant),
      m_rest_length(rest_length)
{
}

void physics::Spring::SetConnectionPointLocal(const math::Point3& connection_point_local)
{
    m_connection_point_local = connection_point_local;
}

const math::Point3& physics::Spring::GetConnectionPointLocal() const
{
    return m_connection_point_local;
}

void physics::Spring::SetOtherBody(physics::RigidBody* other_body)
{
    m_other_body = other_body;
}

physics::RigidBody* physics::Spring::GetOtherBody() const
{
    return m_other_body;
}

void physics::Spring::SetOtherConnectionPointLocal(const math::Point3& other_connection_point_local)
{
    m_other_connection_point_local = other_connection_point_local;
}

const math::Point3& physics::Spring::GetOtherConnectionPointLocal() const
{
    return m_other_connection_point_local;
}

void physics::Spring::SetSpringConstant(physics::real spring_constant)
{
    m_spring_constant = spring_constant;
}

physics::real physics::Spring::GetSpringConstant() const
{
    return m_spring_constant;
}

void physics::Spring::SetRestLength(physics::real rest_length)
{
    m_rest_length = rest_length;
}

physics::real physics::Spring::GetRestLength() const
{
    return m_rest_length;
}

void physics::Spring::UpdateForce(physics::RigidBody& body, physics::real delta_seconds)
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
