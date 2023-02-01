#include <catch2/catch_test_macros.hpp>

#include "physics/particle.h"
#include "physics/particleforcegenerator.h"

TEST_CASE("Default", "Particle")
{
    physics::Particle Particle;
    REQUIRE(Particle.GetPosition() == math::Point3());
    REQUIRE(Particle.GetVelocity() == math::Vector3());
    REQUIRE(Particle.GetAcceleration() == math::Vector3());
    REQUIRE(Particle.GetDamping() == PHYSICS_REALC(0.99));
    REQUIRE(Particle.GetInverseMass() == PHYSICS_REALC(0.0));
    REQUIRE(Particle.HasFiniteMass() == false);
}

TEST_CASE("Setters", "Particle")
{
    physics::Particle Particle;
    Particle.SetPosition(math::Point3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    REQUIRE(Particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    Particle.SetVelocity(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    REQUIRE(Particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    Particle.SetAcceleration(
        math::Vector3(PHYSICS_REALC(7.0), PHYSICS_REALC(8.0), PHYSICS_REALC(9.0)));
    REQUIRE(Particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(7.0), PHYSICS_REALC(8.0), PHYSICS_REALC(9.0)));
    Particle.SetDamping(PHYSICS_REALC(0.5));
    REQUIRE(Particle.GetDamping() == PHYSICS_REALC(0.5));
    Particle.SetMass(1.0f);
    REQUIRE(Particle.GetMass() == PHYSICS_REALC(1.0));
    Particle.SetInverseMass(PHYSICS_REALC(2.0));
    REQUIRE(Particle.GetInverseMass() == PHYSICS_REALC(2.0));
    REQUIRE(Particle.GetMass() == PHYSICS_REALC(0.5));
    REQUIRE(Particle.HasFiniteMass() == true);
}

TEST_CASE("Integration", "Particle")
{
    physics::Particle Particle;
    Particle.SetPosition(math::Point3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    Particle.SetVelocity(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    Particle.SetAcceleration(
        math::Vector3(PHYSICS_REALC(7.0), PHYSICS_REALC(8.0), PHYSICS_REALC(9.0)));
    Particle.SetDamping(PHYSICS_REALC(0.5));
    Particle.SetMass(PHYSICS_REALC(1.0));
    Particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(5.0), PHYSICS_REALC(7.0), PHYSICS_REALC(9.0)));
    REQUIRE(Particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(5.5), PHYSICS_REALC(6.5), PHYSICS_REALC(7.5)));
    REQUIRE(Particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(7.0), PHYSICS_REALC(8.0), PHYSICS_REALC(9.0)));
    REQUIRE(Particle.HasFiniteMass() == true);
}

TEST_CASE("Force", "Particle")
{
    // Write a test for AddForce in Particle.
    physics::Particle Particle;
    Particle.SetMass(PHYSICS_REALC(1.0));
    Particle.SetDamping(PHYSICS_REALC(0.5));
    Particle.SetVelocity(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    Particle.AddForce(math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    Particle.AddForce(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(5.0), PHYSICS_REALC(7.0), PHYSICS_REALC(9.0)));

    // Write a test for Integrate in Particle but with a force accumulator.
    Particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    REQUIRE(Particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(4.5), PHYSICS_REALC(6.0), PHYSICS_REALC(7.5)));
    REQUIRE(Particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(5.0), PHYSICS_REALC(7.0), PHYSICS_REALC(9.0)));
    REQUIRE(Particle.GetForceAccumulator() == math::Vector3());
}

TEST_CASE("Gravity", "Particle")
{
    physics::Particle Particle;
    Particle.SetMass(PHYSICS_REALC(2.0));
    Particle.SetDamping(PHYSICS_REALC(0.5));
    Particle.SetVelocity(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    physics::ParticleGravity Gravity(
        math::Vector3(PHYSICS_REALC(2.0), PHYSICS_REALC(4.0), PHYSICS_REALC(6.0)));
    REQUIRE(Gravity.GetGravity() ==
            math::Vector3(PHYSICS_REALC(2.0), PHYSICS_REALC(4.0), PHYSICS_REALC(6.0)));
    Gravity.UpdateForce(Particle, PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    Particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    REQUIRE(Particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(2.25), PHYSICS_REALC(3.0), PHYSICS_REALC(3.75)));
    REQUIRE(Particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(0.5), PHYSICS_REALC(1.0), PHYSICS_REALC(1.5)));
}

TEST_CASE("Drag", "Particle")
{
    // Write a test for ParticleDrag in Particle.
    physics::Particle Particle;
    Particle.SetMass(PHYSICS_REALC(1.0));
    Particle.SetDamping(PHYSICS_REALC(0.5));
    Particle.SetVelocity(math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    physics::ParticleDrag Drag(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0));
    REQUIRE(Drag.GetK1() == PHYSICS_REALC(1.0));
    REQUIRE(Drag.GetK2() == PHYSICS_REALC(2.0));
    Drag.UpdateForce(Particle, PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-36.0)));
    Particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    REQUIRE(Particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-16.0)));
    REQUIRE(Particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-36.0)));
}

TEST_CASE("ForceRegistry", "Particle")
{
    physics::Particle Particle;
    Particle.SetMass(PHYSICS_REALC(1.0));
    Particle.SetDamping(PHYSICS_REALC(0.5));
    Particle.SetVelocity(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    physics::ParticleGravity Gravity(
        math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    physics::ParticleForceRegistry Registry;
    Registry.Add(&Particle, &Gravity);
    Registry.UpdateForces(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    Particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    REQUIRE(Particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(2.5), PHYSICS_REALC(3.5), PHYSICS_REALC(4.5)));
    REQUIRE(Particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));

    Registry.Clear();
    Registry.UpdateForces(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() == math::Vector3());

    Registry.Add(&Particle, &Gravity);
    Registry.UpdateForces(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    Particle.Integrate(PHYSICS_REALC(1.0));

    Registry.Remove(&Particle, &Gravity);
    Registry.UpdateForces(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() == math::Vector3());
}

TEST_CASE("Spring", "Particle")
{
    // Write a test for ParticleSpring in Particle.
    physics::Particle Particle1;
    Particle1.SetMass(PHYSICS_REALC(1.0));
    Particle1.SetDamping(PHYSICS_REALC(0.5));
    Particle1.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    Particle1.SetVelocity(math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    physics::Particle Particle2;
    Particle2.SetMass(PHYSICS_REALC(1.0));
    Particle2.SetDamping(PHYSICS_REALC(0.5));
    Particle2.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    Particle2.SetVelocity(math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    physics::ParticleSpring SpringGen(&Particle1, PHYSICS_REALC(2.0), PHYSICS_REALC(2.0));
    REQUIRE(SpringGen.GetOther() == &Particle1);
    REQUIRE(SpringGen.GetSpringConstant() == PHYSICS_REALC(2.0));
    REQUIRE(SpringGen.GetRestLength() == PHYSICS_REALC(2.0));
    SpringGen.UpdateForce(Particle2, PHYSICS_REALC(1.0));
    REQUIRE(Particle2.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-4.0)));
    Particle2.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(Particle2.GetPosition() ==
            math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    REQUIRE(Particle2.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-2.0)));
    REQUIRE(Particle2.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-4.0)));
    SpringGen.SetOther(&Particle2);
    REQUIRE(SpringGen.GetOther() == &Particle2);
    SpringGen.SetSpringConstant(PHYSICS_REALC(4.0));
    REQUIRE(SpringGen.GetSpringConstant() == PHYSICS_REALC(4.0));
    SpringGen.SetRestLength(PHYSICS_REALC(4.0));
    REQUIRE(SpringGen.GetRestLength() == PHYSICS_REALC(4.0));
}

TEST_CASE("AnchoredSpring", "Particle")
{
    // Write a test for ParticleAnchoredSpring in Particle.
    physics::Particle Particle;
    Particle.SetMass(PHYSICS_REALC(1.0));
    Particle.SetDamping(PHYSICS_REALC(0.5));
    Particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    Particle.SetVelocity(math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    physics::ParticleAnchoredSpring SpringGen(
        math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)),
        PHYSICS_REALC(2.0),
        PHYSICS_REALC(2.0));
    REQUIRE(SpringGen.GetAnchor() ==
            math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    REQUIRE(SpringGen.GetSpringConstant() == PHYSICS_REALC(2.0));
    REQUIRE(SpringGen.GetRestLength() == PHYSICS_REALC(2.0));
    SpringGen.UpdateForce(Particle, PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    Particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    REQUIRE(Particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(2.0)));
    REQUIRE(Particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    SpringGen.SetAnchor(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(2.0)));
    REQUIRE(SpringGen.GetAnchor() ==
            math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(2.0)));
    SpringGen.SetSpringConstant(PHYSICS_REALC(4.0));
    REQUIRE(SpringGen.GetSpringConstant() == PHYSICS_REALC(4.0));
    SpringGen.SetRestLength(PHYSICS_REALC(4.0));
    REQUIRE(SpringGen.GetRestLength() == PHYSICS_REALC(4.0));
}
