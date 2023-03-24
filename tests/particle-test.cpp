#include <catch2/catch_test_macros.hpp>

#include "physics/particle-force-generator.h"
#include "physics/particle.h"

TEST_CASE("Default particle constructor", "[particle][creation]")
{
    const Physics::Particle particle;
    REQUIRE(particle.GetPosition() == math::Point3());
    REQUIRE(particle.GetVelocity() == math::Vector3());
    REQUIRE(particle.GetAcceleration() == math::Vector3());
    REQUIRE(particle.GetDamping() == PHYSICS_REALC(0.99));
    REQUIRE(particle.GetInverseMass() == PHYSICS_REALC(0.0));
    REQUIRE(particle.HasFiniteMass() == false);
}

TEST_CASE("Particle setters", "[particle][creation]")
{
    Physics::Particle particle;
    particle.SetPosition(math::Point3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    REQUIRE(particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    particle.SetVelocity(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    REQUIRE(particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    particle.SetAcceleration(
        math::Vector3(PHYSICS_REALC(7.0), PHYSICS_REALC(8.0), PHYSICS_REALC(9.0)));
    REQUIRE(particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(7.0), PHYSICS_REALC(8.0), PHYSICS_REALC(9.0)));
    particle.SetDamping(PHYSICS_REALC(0.5));
    REQUIRE(particle.GetDamping() == PHYSICS_REALC(0.5));
    particle.SetMass(1.0f);
    REQUIRE(particle.GetMass() == PHYSICS_REALC(1.0));
    particle.SetInverseMass(PHYSICS_REALC(2.0));
    REQUIRE(particle.GetInverseMass() == PHYSICS_REALC(2.0));
    REQUIRE(particle.GetMass() == PHYSICS_REALC(0.5));
    REQUIRE(particle.HasFiniteMass() == true);
}

TEST_CASE("Particle integration", "[particle]")
{
    Physics::Particle particle;
    particle.SetPosition(math::Point3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    particle.SetVelocity(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    particle.SetAcceleration(
        math::Vector3(PHYSICS_REALC(7.0), PHYSICS_REALC(8.0), PHYSICS_REALC(9.0)));
    particle.SetDamping(PHYSICS_REALC(0.5));
    particle.SetMass(PHYSICS_REALC(1.0));
    particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(5.0), PHYSICS_REALC(7.0), PHYSICS_REALC(9.0)));
    REQUIRE(particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(5.5), PHYSICS_REALC(6.5), PHYSICS_REALC(7.5)));
    REQUIRE(particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(7.0), PHYSICS_REALC(8.0), PHYSICS_REALC(9.0)));
    REQUIRE(particle.HasFiniteMass() == true);
}

TEST_CASE("Applying force to particle", "[particle]")
{
    // Setup particle.
    Physics::Particle particle;
    particle.SetMass(PHYSICS_REALC(1.0));
    particle.SetDamping(PHYSICS_REALC(0.5));
    particle.SetVelocity(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));

    // Add a couple of forces and verify the force accumulator.
    particle.AddForce(math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    particle.AddForce(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    REQUIRE(particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(5.0), PHYSICS_REALC(7.0), PHYSICS_REALC(9.0)));

    // Integrate the particle and verify the new position, velocity and acceleration.
    particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    REQUIRE(particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(4.5), PHYSICS_REALC(6.0), PHYSICS_REALC(7.5)));
    REQUIRE(particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(5.0), PHYSICS_REALC(7.0), PHYSICS_REALC(9.0)));
    REQUIRE(particle.GetForceAccumulator() == math::Vector3::Zero);
}

TEST_CASE("Gravity particle generator", "[particle][forcegenerator]")
{
    // Setup particle.
    Physics::Particle particle;
    particle.SetMass(PHYSICS_REALC(2.0));
    particle.SetDamping(PHYSICS_REALC(0.5));
    particle.SetVelocity(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));

    // Add a gravity force and verify the force accumulator.
    Physics::ParticleGravity gravity(
        math::Vector3(PHYSICS_REALC(2.0), PHYSICS_REALC(4.0), PHYSICS_REALC(6.0)));
    REQUIRE(gravity.GetGravity() ==
            math::Vector3(PHYSICS_REALC(2.0), PHYSICS_REALC(4.0), PHYSICS_REALC(6.0)));
    gravity.UpdateForce(particle, PHYSICS_REALC(1.0));
    REQUIRE(particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));

    // Integrate the particle and verify the new position, velocity and acceleration.
    particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    REQUIRE(particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(2.25), PHYSICS_REALC(3.0), PHYSICS_REALC(3.75)));
    REQUIRE(particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(0.5), PHYSICS_REALC(1.0), PHYSICS_REALC(1.5)));

    // Verify setters work.
    gravity.SetGravity(math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    REQUIRE(gravity.GetGravity() ==
            math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
}

TEST_CASE("Applying drag force to particle", "[particle][forcegenerator]")
{
    // Setup particle.
    Physics::Particle particle;
    particle.SetMass(PHYSICS_REALC(1.0));
    particle.SetDamping(PHYSICS_REALC(0.5));
    particle.SetVelocity(math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));

    // Add a drag force and verify the force accumulator.
    Physics::ParticleDrag drag(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0));
    REQUIRE(drag.GetK1() == PHYSICS_REALC(1.0));
    REQUIRE(drag.GetK2() == PHYSICS_REALC(2.0));
    drag.UpdateForce(particle, PHYSICS_REALC(1.0));
    REQUIRE(particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-36.0)));

    // Integrate the particle and verify the new position, velocity and acceleration.
    particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    REQUIRE(particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-16.0)));
    REQUIRE(particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-36.0)));

    // Verify setters work.
    drag.SetK1(PHYSICS_REALC(2.0));
    drag.SetK2(PHYSICS_REALC(3.0));
    REQUIRE(drag.GetK1() == PHYSICS_REALC(2.0));
    REQUIRE(drag.GetK2() == PHYSICS_REALC(3.0));
}

TEST_CASE("Particle force generator", "[particle][forcegenerator]")
{
    // Setup particle.
    Physics::Particle particle;
    particle.SetMass(PHYSICS_REALC(1.0));
    particle.SetDamping(PHYSICS_REALC(0.5));
    particle.SetVelocity(math::Vector3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    Physics::ParticleGravity gravity(
        math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));

    // Add a force registry, apply forces using it and verify the force accumulator.
    Physics::ParticleForceRegistry registry;
    registry.Add(&particle, &gravity);
    registry.UpdateForces(PHYSICS_REALC(1.0));
    REQUIRE(particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));

    // Integrate the particle and verify the new position, velocity and acceleration.
    particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(4.0), PHYSICS_REALC(5.0), PHYSICS_REALC(6.0)));
    REQUIRE(particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(2.5), PHYSICS_REALC(3.5), PHYSICS_REALC(4.5)));
    REQUIRE(particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));

    // Clear the force registry, apply forces using it and verify the force accumulator.
    registry.Clear();
    registry.UpdateForces(PHYSICS_REALC(1.0));
    REQUIRE(particle.GetForceAccumulator() == math::Vector3());

    // Add again.
    registry.Add(&particle, &gravity);
    registry.UpdateForces(PHYSICS_REALC(1.0));
    REQUIRE(particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(1.0), PHYSICS_REALC(2.0), PHYSICS_REALC(3.0)));
    particle.Integrate(PHYSICS_REALC(1.0));

    // Remove specific pair and verify the force accumulator.
    registry.Remove(&particle, &gravity);
    registry.UpdateForces(PHYSICS_REALC(1.0));
    REQUIRE(particle.GetForceAccumulator() == math::Vector3());
}

TEST_CASE("Applying spring force to particle", "[particle][forcegenerator]")
{
    // Setup first particle.
    Physics::Particle particle1;
    particle1.SetMass(PHYSICS_REALC(1.0));
    particle1.SetDamping(PHYSICS_REALC(0.5));
    particle1.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    particle1.SetVelocity(
        math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));

    // Setup second particle.
    Physics::Particle particle2;
    particle2.SetMass(PHYSICS_REALC(1.0));
    particle2.SetDamping(PHYSICS_REALC(0.5));
    particle2.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    particle2.SetVelocity(
        math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));

    // Setup spring.
    Physics::ParticleSpring spring_gen(&particle1, PHYSICS_REALC(2.0), PHYSICS_REALC(2.0));

    // Verify that getters work.
    REQUIRE(spring_gen.GetOther() == &particle1);
    REQUIRE(spring_gen.GetSpringConstant() == PHYSICS_REALC(2.0));
    REQUIRE(spring_gen.GetRestLength() == PHYSICS_REALC(2.0));

    // Checks if force is correct when spring length is larger than rest length.
    spring_gen.UpdateForce(particle2, PHYSICS_REALC(1.0));
    REQUIRE(particle2.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-4.0)));

    // Checks if force is correct when spring length is shorter than rest length.
    particle2.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(1.0)));
    spring_gen.UpdateForce(particle2, PHYSICS_REALC(1.0));
    REQUIRE(particle2.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-2.0)));

    // Checks if force is correct when spring is at rest length.
    particle2.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(2.0)));
    spring_gen.UpdateForce(particle2, PHYSICS_REALC(1.0));
    REQUIRE(particle2.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-2.0)));

    // Checks setters.
    spring_gen.SetOther(&particle2);
    REQUIRE(spring_gen.GetOther() == &particle2);
    spring_gen.SetSpringConstant(PHYSICS_REALC(4.0));
    REQUIRE(spring_gen.GetSpringConstant() == PHYSICS_REALC(4.0));
    spring_gen.SetRestLength(PHYSICS_REALC(4.0));
    REQUIRE(spring_gen.GetRestLength() == PHYSICS_REALC(4.0));
}

TEST_CASE("Applying anchored spring force to particle", "[particle][forcegenerator]")
{
    // Write a test for ParticleAnchoredSpring in Particle.
    Physics::Particle particle;
    particle.SetMass(PHYSICS_REALC(1.0));
    particle.SetDamping(PHYSICS_REALC(0.5));
    particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    particle.SetVelocity(math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    Physics::ParticleAnchoredSpring spring_gen(
        math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)),
        PHYSICS_REALC(2.0), PHYSICS_REALC(2.0));
    REQUIRE(spring_gen.GetAnchor() ==
            math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    REQUIRE(spring_gen.GetSpringConstant() == PHYSICS_REALC(2.0));
    REQUIRE(spring_gen.GetRestLength() == PHYSICS_REALC(2.0));
    spring_gen.UpdateForce(particle, PHYSICS_REALC(1.0));
    REQUIRE(particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    particle.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(particle.GetPosition() ==
            math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    REQUIRE(particle.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(2.0)));
    REQUIRE(particle.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    spring_gen.SetAnchor(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(2.0)));
    REQUIRE(spring_gen.GetAnchor() ==
            math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(2.0)));
    spring_gen.SetSpringConstant(PHYSICS_REALC(4.0));
    REQUIRE(spring_gen.GetSpringConstant() == PHYSICS_REALC(4.0));
    spring_gen.SetRestLength(PHYSICS_REALC(4.0));
    REQUIRE(spring_gen.GetRestLength() == PHYSICS_REALC(4.0));
}

TEST_CASE("Applying bungee force to particle", "[particle][forcegenerator]")
{
    // Setup first particle at origin with no velocity.
    Physics::Particle particle;
    particle.SetMass(PHYSICS_REALC(1.0));
    particle.SetDamping(PHYSICS_REALC(0.5));
    particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    particle.SetVelocity(math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));

    // Setup second particle which we will apply the force to.
    Physics::Particle particle2;
    particle2.SetMass(PHYSICS_REALC(1.0));
    particle2.SetDamping(PHYSICS_REALC(0.5));
    particle2.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    particle2.SetVelocity(
        math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));

    // Setup the bungee spring generator.
    Physics::ParticleBungee spring_gen(&particle, PHYSICS_REALC(2.0), PHYSICS_REALC(2.0));
    REQUIRE(spring_gen.GetOther() == &particle);
    REQUIRE(spring_gen.GetSpringConstant() == PHYSICS_REALC(2.0));
    REQUIRE(spring_gen.GetRestLength() == PHYSICS_REALC(2.0));

    // Apply bungee force to the second particle. Verify that the force is applied since the
    // particle has extended the spring beyond its rest length.
    spring_gen.UpdateForce(particle2, PHYSICS_REALC(1.0));
    REQUIRE(particle2.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-4.0)));
    particle2.Integrate(PHYSICS_REALC(1.0));
    REQUIRE(particle2.GetPosition() ==
            math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(4.0)));
    REQUIRE(particle2.GetVelocity() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-2.0)));
    REQUIRE(particle2.GetAcceleration() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(-4.0)));

    // Verify setters.
    spring_gen.SetOther(&particle2);
    REQUIRE(spring_gen.GetOther() == &particle2);
    spring_gen.SetSpringConstant(PHYSICS_REALC(4.0));
    REQUIRE(spring_gen.GetSpringConstant() == PHYSICS_REALC(4.0));
    spring_gen.SetRestLength(PHYSICS_REALC(4.0));
    REQUIRE(spring_gen.GetRestLength() == PHYSICS_REALC(4.0));

    // Verify that no force is applied if the particle is within the rest length.
    spring_gen.SetOther(&particle);
    spring_gen.UpdateForce(particle2, PHYSICS_REALC(1.0));
    REQUIRE(particle2.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
}

TEST_CASE("Applying buoyancy force to the particle", "[particle][forcegenerator]")
{
    // Setup particle.
    Physics::Particle particle;
    particle.SetMass(PHYSICS_REALC(1.0));
    particle.SetDamping(PHYSICS_REALC(0.5));
    particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    particle.SetVelocity(math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));

    // Setup buoyancy generator.
    Physics::ParticleBuoyancy buoyancy_gen(PHYSICS_REALC(2.0), PHYSICS_REALC(2.0),
                                          PHYSICS_REALC(2.0), PHYSICS_REALC(2.0));
    REQUIRE(buoyancy_gen.GetMaxDepth() == PHYSICS_REALC(2.0));
    REQUIRE(buoyancy_gen.GetVolume() == PHYSICS_REALC(2.0));
    REQUIRE(buoyancy_gen.GetWaterHeight() == PHYSICS_REALC(2.0));
    REQUIRE(buoyancy_gen.GetLiquidDensity() == PHYSICS_REALC(2.0));

    // Verify that there is no force when object is fully out of the water.
    particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(5.0), PHYSICS_REALC(0.0)));
    buoyancy_gen.UpdateForce(particle, PHYSICS_REALC(1.0));
    REQUIRE(particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));
    particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(4.0), PHYSICS_REALC(0.0)));
    buoyancy_gen.UpdateForce(particle, PHYSICS_REALC(1.0));
    REQUIRE(particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(0.0), PHYSICS_REALC(0.0)));

    // Fully submerged
    particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(-2.0), PHYSICS_REALC(0.0)));
    buoyancy_gen.UpdateForce(particle, PHYSICS_REALC(1.0));
    REQUIRE(particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(4.0), PHYSICS_REALC(0.0)));
    particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(-4.0), PHYSICS_REALC(0.0)));
    buoyancy_gen.UpdateForce(particle, PHYSICS_REALC(1.0));
    REQUIRE(particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(8.0), PHYSICS_REALC(0.0)));

    // Partially submerged
    particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(2.0), PHYSICS_REALC(0.0)));
    buoyancy_gen.UpdateForce(particle, PHYSICS_REALC(1.0));
    REQUIRE(particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(10.0), PHYSICS_REALC(0.0)));
    particle.SetPosition(math::Point3(PHYSICS_REALC(0.0), PHYSICS_REALC(1.0), PHYSICS_REALC(0.0)));
    buoyancy_gen.UpdateForce(particle, PHYSICS_REALC(1.0));
    REQUIRE(particle.GetForceAccumulator() ==
            math::Vector3(PHYSICS_REALC(0.0), PHYSICS_REALC(13.0), PHYSICS_REALC(0.0)));

    // Verify setters.
    buoyancy_gen.SetLiquidDensity(PHYSICS_REALC(4.0));
    REQUIRE(buoyancy_gen.GetLiquidDensity() == PHYSICS_REALC(4.0));
    buoyancy_gen.SetMaxDepth(PHYSICS_REALC(4.0));
    REQUIRE(buoyancy_gen.GetMaxDepth() == PHYSICS_REALC(4.0));
    buoyancy_gen.SetVolume(PHYSICS_REALC(4.0));
    REQUIRE(buoyancy_gen.GetVolume() == PHYSICS_REALC(4.0));
    buoyancy_gen.SetWaterHeight(PHYSICS_REALC(4.0));
    REQUIRE(buoyancy_gen.GetWaterHeight() == PHYSICS_REALC(4.0));
}
