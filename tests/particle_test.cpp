#include <catch2/catch_test_macros.hpp>

#include "physics/particle.h"

TEST_CASE("Default", "Particle")
{
    physics::Particle Particle;
    REQUIRE(Particle.GetPosition() == math::Point3());
    REQUIRE(Particle.GetVelocity() == math::Vector3());
    REQUIRE(Particle.GetAcceleration() == math::Vector3());
    REQUIRE(Particle.GetDamping() == PHYSICS_REALC(0.99));
    REQUIRE(Particle.GetInverseMass() == PHYSICS_REALC(0.0));
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
