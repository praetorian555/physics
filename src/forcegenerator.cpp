#include "physics/forcegenerator.h"

#include "physics/body.h"

void physics::Gravity::UpdateForce(RigidBody& Body, real DeltaSeconds)
{
    PHYSICS_UNUSED(DeltaSeconds);
    if (!Body.HasFiniteMass())
    {
        return;
    }
    Body.AddForce(m_Gravity * Body.GetMass());
}

physics::Spring::Spring(const math::Point3& ConnectionPointLocal,
                        physics::RigidBody* OtherBody,
                        const math::Point3& OtherConnectionPointLocal,
                        physics::real SpringConstant,
                        physics::real RestLength)
    : m_ConnectionPointLocal(ConnectionPointLocal),
      m_OtherBody(OtherBody),
      m_OtherConnectionPointLocal(OtherConnectionPointLocal),
      m_SpringConstant(SpringConstant),
      m_RestLength(RestLength)
{
}

void physics::Spring::SetConnectionPointLocal(const math::Point3& ConnectionPointLocal)
{
    m_ConnectionPointLocal = ConnectionPointLocal;
}

const math::Point3& physics::Spring::GetConnectionPointLocal() const
{
    return m_ConnectionPointLocal;
}

void physics::Spring::SetOtherBody(physics::RigidBody* OtherBody)
{
    m_OtherBody = OtherBody;
}

physics::RigidBody* physics::Spring::GetOtherBody() const
{
    return m_OtherBody;
}

void physics::Spring::SetOtherConnectionPointLocal(const math::Point3& OtherConnectionPointLocal)
{
    m_OtherConnectionPointLocal = OtherConnectionPointLocal;
}

const math::Point3& physics::Spring::GetOtherConnectionPointLocal() const
{
    return m_OtherConnectionPointLocal;
}

void physics::Spring::SetSpringConstant(physics::real SpringConstant)
{
    m_SpringConstant = SpringConstant;
}

physics::real physics::Spring::GetSpringConstant() const
{
    return m_SpringConstant;
}

void physics::Spring::SetRestLength(physics::real RestLength)
{
    m_RestLength = RestLength;
}

physics::real physics::Spring::GetRestLength() const
{
    return m_RestLength;
}

void physics::Spring::UpdateForce(physics::RigidBody& Body, physics::real DeltaSeconds)
{
    PHYSICS_UNUSED(DeltaSeconds);

    const math::Point3 ConnectionPointWorld = Body.GetTransform()(m_ConnectionPointLocal);
    const math::Point3 OtherConnectionPointWorld =
        m_OtherBody->GetTransform()(m_OtherConnectionPointLocal);

    math::Vector3 LengthVector = ConnectionPointWorld - OtherConnectionPointWorld;
    const real Length = LengthVector.Length();
    const real SpringForce = m_SpringConstant * (Length - m_RestLength);
    LengthVector = math::Normalize(LengthVector);

    Body.AddForce(-LengthVector * SpringForce);
}
