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
    // Setup particle.
    physics::Particle Particle;
    Particle.SetMass(PHYSICS_REALC(1.0));
    Particle.SetDamping(PHYSICS_REALC(0.5));
    Particle.SetVelocity(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));

    // Add a couple of forces and verify the force accumulator.
    Particle.AddForce(math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    Particle.AddForce(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(5.0), PHYSICS_REALC(7.0), PHYSICS_REALC(9.0)));

    // Integrate the particle and verify the new position, velocity and acceleration.
    Particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    REQUIRE(Particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(4.5), PHYSICS_REALC(6.0), PHYSICS_REALC(7.5)));
    REQUIRE(Particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(5.0), PHYSICS_REALC(7.0), PHYSICS_REALC(9.0)));
    REQUIRE(Particle.GetForceAccumulator() == math::Vector3::Zero);
}

TEST_CASE("Gravity", "Particle")
{
    // Setup particle.
    physics::Particle Particle;
    Particle.SetMass(PHYSICS_REALC(2.0));
    Particle.SetDamping(PHYSICS_REALC(0.5));
    Particle.SetVelocity(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));

    // Add a gravity force and verify the force accumulator.
    physics::ParticleGravity Gravity(
        math::Vector3(PHYSICS_REALC(2.0), PHYSICS_REALC(4.0), PHYSICS_REALC(6.0)));
    REQUIRE(Gravity.GetGravity() ==
            math::Vector3(PHYSICS_REALC(2.0), PHYSICS_REALC(4.0), PHYSICS_REALC(6.0)));
    Gravity.UpdateForce(Particle, PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));

    // Integrate the particle and verify the new position, velocity and acceleration.
    Particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    REQUIRE(Particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(2.25), PHYSICS_REALC(3.0), PHYSICS_REALC(3.75)));
    REQUIRE(Particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(0.5), PHYSICS_REALC(1.0), PHYSICS_REALC(1.5)));

    // Verify setters work.
    Gravity.SetGravity(math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    REQUIRE(Gravity.GetGravity() ==
            math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
}

TEST_CASE("Drag", "Particle")
{
    // Setup particle.
    physics::Particle Particle;
    Particle.SetMass(PHYSICS_REALC(1.0));
    Particle.SetDamping(PHYSICS_REALC(0.5));
    Particle.SetVelocity(math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));

    // Add a drag force and verify the force accumulator.
    physics::ParticleDrag Drag(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0));
    REQUIRE(Drag.GetK1() == PHYSICS_REALC(1.0));
    REQUIRE(Drag.GetK2() == PHYSICS_REALC(2.0));
    Drag.UpdateForce(Particle, PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-36.0)));

    // Integrate the particle and verify the new position, velocity and acceleration.
    Particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    REQUIRE(Particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-16.0)));
    REQUIRE(Particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-36.0)));

    // Verify setters work.
    Drag.SetK1(PHYSICS_REALC(2.0));
    Drag.SetK2(PHYSICS_REALC(3.0));
    REQUIRE(Drag.GetK1() == PHYSICS_REALC(2.0));
    REQUIRE(Drag.GetK2() == PHYSICS_REALC(3.0));
}

TEST_CASE("ForceRegistry", "Particle")
{
    // Setup particle.
    physics::Particle Particle;
    Particle.SetMass(PHYSICS_REALC(1.0));
    Particle.SetDamping(PHYSICS_REALC(0.5));
    Particle.SetVelocity(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    physics::ParticleGravity Gravity(
        math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));

    // Add a force registry, apply forces using it and verify the force accumulator.
    physics::ParticleForceRegistry Registry;
    Registry.Add(&Particle, &Gravity);
    Registry.UpdateForces(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));

    // Integrate the particle and verify the new position, velocity and acceleration.
    Particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    REQUIRE(Particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(2.5), PHYSICS_REALC(3.5), PHYSICS_REALC(4.5)));
    REQUIRE(Particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));

    // Clear the force registry, apply forces using it and verify the force accumulator.
    Registry.Clear();
    Registry.UpdateForces(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() == math::Vector3());

    // Add again.
    Registry.Add(&Particle, &Gravity);
    Registry.UpdateForces(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    Particle.Integrate(PHYSICS_REALC(1.0));

    // Remove specific pair and verify the force accumulator.
    Registry.Remove(&Particle, &Gravity);
    Registry.UpdateForces(PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() == math::Vector3());
}

TEST_CASE("Spring", "Particle")
{
    // Setup first particle.
    physics::Particle Particle1;
    Particle1.SetMass(PHYSICS_REALC(1.0));
    Particle1.SetDamping(PHYSICS_REALC(0.5));
    Particle1.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    Particle1.SetVelocity(
        math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));

    // Setup second particle.
    physics::Particle Particle2;
    Particle2.SetMass(PHYSICS_REALC(1.0));
    Particle2.SetDamping(PHYSICS_REALC(0.5));
    Particle2.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    Particle2.SetVelocity(
        math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));

    // Setup spring.
    physics::ParticleSpring SpringGen(&Particle1, PHYSICS_REALC(2.0), PHYSICS_REALC(2.0));

    // Verify that getters work.
    REQUIRE(SpringGen.GetOther() == &Particle1);
    REQUIRE(SpringGen.GetSpringConstant() == PHYSICS_REALC(2.0));
    REQUIRE(SpringGen.GetRestLength() == PHYSICS_REALC(2.0));

    // Checks if force is correct when spring length is larger than rest length.
    SpringGen.UpdateForce(Particle2, PHYSICS_REALC(1.0));
    REQUIRE(Particle2.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-4.0)));

    // Checks if force is correct when spring length is shorter than rest length.
    Particle2.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(1.0)));
    SpringGen.UpdateForce(Particle2, PHYSICS_REALC(1.0));
    REQUIRE(Particle2.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-2.0)));

    // Checks if force is correct when spring is at rest length.
    Particle2.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(2.0)));
    SpringGen.UpdateForce(Particle2, PHYSICS_REALC(1.0));
    REQUIRE(Particle2.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-2.0)));

    // Checks setters.
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
        PHYSICS_REALC(2.0), PHYSICS_REALC(2.0));
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

TEST_CASE("BungeeSpring", "Particle")
{
    // Setup first particle at origin with no velocity.
    physics::Particle Particle;
    Particle.SetMass(PHYSICS_REALC(1.0));
    Particle.SetDamping(PHYSICS_REALC(0.5));
    Particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    Particle.SetVelocity(math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));

    // Setup second particle which we will apply the force to.
    physics::Particle Particle2;
    Particle2.SetMass(PHYSICS_REALC(1.0));
    Particle2.SetDamping(PHYSICS_REALC(0.5));
    Particle2.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    Particle2.SetVelocity(
        math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));

    // Setup the bungee spring generator.
    physics::ParticleBungee SpringGen(&Particle, PHYSICS_REALC(2.0), PHYSICS_REALC(2.0));
    REQUIRE(SpringGen.GetOther() == &Particle);
    REQUIRE(SpringGen.GetSpringConstant() == PHYSICS_REALC(2.0));
    REQUIRE(SpringGen.GetRestLength() == PHYSICS_REALC(2.0));

    // Apply bungee force to the second particle. Verify that the force is applied since the
    // particle has extended the spring beyond its rest length.
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

    // Verify setters.
    SpringGen.SetOther(&Particle2);
    REQUIRE(SpringGen.GetOther() == &Particle2);
    SpringGen.SetSpringConstant(PHYSICS_REALC(4.0));
    REQUIRE(SpringGen.GetSpringConstant() == PHYSICS_REALC(4.0));
    SpringGen.SetRestLength(PHYSICS_REALC(4.0));
    REQUIRE(SpringGen.GetRestLength() == PHYSICS_REALC(4.0));

    // Verify that no force is applied if the particle is within the rest length.
    SpringGen.SetOther(&Particle);
    SpringGen.UpdateForce(Particle2, PHYSICS_REALC(1.0));
    REQUIRE(Particle2.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
}

TEST_CASE("Buoyancy", "Particle")
{
    // Setup particle.
    physics::Particle Particle;
    Particle.SetMass(PHYSICS_REALC(1.0));
    Particle.SetDamping(PHYSICS_REALC(0.5));
    Particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    Particle.SetVelocity(math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));

    // Setup buoyancy generator.
    physics::ParticleBuoyancy BuoyancyGen(PHYSICS_REALC(2.0), PHYSICS_REALC(2.0),
                                          PHYSICS_REALC(2.0), PHYSICS_REALC(2.0));
    REQUIRE(BuoyancyGen.GetMaxDepth() == PHYSICS_REALC(2.0));
    REQUIRE(BuoyancyGen.GetVolume() == PHYSICS_REALC(2.0));
    REQUIRE(BuoyancyGen.GetWaterHeight() == PHYSICS_REALC(2.0));
    REQUIRE(BuoyancyGen.GetLiquidDensity() == PHYSICS_REALC(2.0));

    // Verify that there is no force when object is fully out of the water.
    Particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(5.0), PHYSICS_REALC(0.0)));
    BuoyancyGen.UpdateForce(Particle, PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    Particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(4.0), PHYSICS_REALC(0.0)));
    BuoyancyGen.UpdateForce(Particle, PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));

    // Fully submerged
    Particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(-2.0), PHYSICS_REALC(0.0)));
    BuoyancyGen.UpdateForce(Particle, PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(4.0), PHYSICS_REALC(0.0)));
    Particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(-4.0), PHYSICS_REALC(0.0)));
    BuoyancyGen.UpdateForce(Particle, PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(8.0), PHYSICS_REALC(0.0)));

    // Partially submerged
    Particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(2.0), PHYSICS_REALC(0.0)));
    BuoyancyGen.UpdateForce(Particle, PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(10.0), PHYSICS_REALC(0.0)));
    Particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(1.0), PHYSICS_REALC(0.0)));
    BuoyancyGen.UpdateForce(Particle, PHYSICS_REALC(1.0));
    REQUIRE(Particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(13.0), PHYSICS_REALC(0.0)));

    // Verify setters.
    BuoyancyGen.SetLiquidDensity(PHYSICS_REALC(4.0));
    REQUIRE(BuoyancyGen.GetLiquidDensity() == PHYSICS_REALC(4.0));
    BuoyancyGen.SetMaxDepth(PHYSICS_REALC(4.0));
    REQUIRE(BuoyancyGen.GetMaxDepth() == PHYSICS_REALC(4.0));
    BuoyancyGen.SetVolume(PHYSICS_REALC(4.0));
    REQUIRE(BuoyancyGen.GetVolume() == PHYSICS_REALC(4.0));
    BuoyancyGen.SetWaterHeight(PHYSICS_REALC(4.0));
    REQUIRE(BuoyancyGen.GetWaterHeight() == PHYSICS_REALC(4.0));
}
