#include <catch2/catch_test_macros.hpp>

#include "physics/particle-contacts.h"
#include "physics/particle.h"

TEST_CASE("Creation of particle contact", "[creation][particle][contact]")
{
    SECTION("Create with two particles")
    {
        physics::Particle particle1;
        physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 5, 3});
        physics::ParticleContact contact(&particle1, &particle2, 0.5f, 1.0f);
        REQUIRE(contact.IsValid());
        REQUIRE(contact.GetMainParticle() == &particle1);
        REQUIRE(contact.GetOtherParticle() == &particle2);
        REQUIRE(contact.GetRestitution() == 0.5f);
        REQUIRE(contact.GetPenetration() == 1.0f);
        REQUIRE(contact.GetContactNormal() ==
                math::Normalize(particle1.GetPosition() - particle2.GetPosition()));

        contact = physics::ParticleContact(&particle2, nullptr, 0.5f, 1.0f);
        REQUIRE(!contact.IsValid());
        contact = physics::ParticleContact(nullptr, &particle2, 0.5f, 1.0f);
        REQUIRE(!contact.IsValid());
        contact = physics::ParticleContact(&particle1, &particle2, -0.5f, 1.0f);
        REQUIRE(!contact.IsValid());
        contact = physics::ParticleContact(&particle1, &particle2, 1.5f, 1.0f);
        REQUIRE(!contact.IsValid());
    }

    SECTION("Create with one particle")
    {
        physics::Particle particle1;
        physics::ParticleContact contact(&particle1, 0.5f, 2.0f, math::Vector3{0, 0, 3});
        REQUIRE(contact.IsValid());
        REQUIRE(contact.GetMainParticle() == &particle1);
        REQUIRE(contact.GetOtherParticle() == nullptr);
        REQUIRE(contact.GetRestitution() == 0.5f);
        REQUIRE(contact.GetPenetration() == 2.0f);
        REQUIRE(contact.GetContactNormal() == math::Normalize(math::Vector3{0, 0, 1}));

        contact = physics::ParticleContact(nullptr, 0.5f, 1.0f, math::Vector3{0, 0, 3});
        REQUIRE(!contact.IsValid());
        contact = physics::ParticleContact(&particle1, -0.5f, 1.0f, math::Vector3{0, 0, 3});
        REQUIRE(!contact.IsValid());
        contact = physics::ParticleContact(&particle1, 1.5f, 1.0f, math::Vector3{0, 0, 3});
        REQUIRE(!contact.IsValid());
        contact = physics::ParticleContact(&particle1, 0.5f, 1.0f, math::Vector3{0, 0, 0});
        REQUIRE(!contact.IsValid());
    }
}

TEST_CASE("Resolving a contact", "[particle][contact]")
{
    SECTION("Resolve velocity when approaching")
    {
        physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        particle1.SetVelocity(math::Vector3{0, 0, 1});
        physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(4.0f);
        particle2.SetVelocity(math::Vector3{0, 0, -1});
        physics::ParticleContact contact(&particle1, &particle2, 0.5f, 1.0f);
        contact.Resolve(1.0f);
        REQUIRE(particle1.GetVelocity() == math::Vector3{0, 0, 0.0f});
        REQUIRE(particle2.GetVelocity() == math::Vector3{0, 0, 1.0f});
    }
    SECTION("Resolve velocity when separating")
    {
        physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        particle1.SetVelocity(math::Vector3{0, 0, -1});
        physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(4.0f);
        particle2.SetVelocity(math::Vector3{0, 0, 1});
        physics::ParticleContact contact(&particle1, &particle2, 0.5f, 1.0f);
        contact.Resolve(1.0f);
        REQUIRE(particle1.GetVelocity() == math::Vector3{0, 0, -1.0f});
        REQUIRE(particle2.GetVelocity() == math::Vector3{0, 0, 1.0f});
    }
    SECTION("Resolve velocity when immovable objects")
    {
        physics::Particle particle1;
        particle1.SetInverseMass(0.0f);
        particle1.SetVelocity(math::Vector3{0, 0, -1});
        physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(0.0f);
        particle2.SetVelocity(math::Vector3{0, 0, 1});
        physics::ParticleContact contact(&particle1, &particle2, 0.5f, 1.0f);
        contact.Resolve(1.0f);
        REQUIRE(particle1.GetVelocity() == math::Vector3{0, 0, -1.0f});
        REQUIRE(particle2.GetVelocity() == math::Vector3{0, 0, 1.0f});
    }

    SECTION("Resolve velocity with rest contact")
    {
        physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        particle1.SetVelocity(math::Vector3{0, 0, -1});
        particle1.SetAcceleration(math::Vector3{0, 0, -1});
        physics::ParticleContact contact(&particle1, 1.0f, 1.0f, math::Vector3{0, 0, 1});
        contact.Resolve(1.0f);
        REQUIRE(particle1.GetVelocity() == math::Vector3{0, 0, 0.0f});
    }

    SECTION("Resolve interpenetration")
    {
        physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(4.0f);
        physics::ParticleContact contact(&particle1, &particle2, 0.5f, 3.0f);
        contact.Resolve(1.0f);
        REQUIRE(particle1.GetPosition() == math::Point3{0, 0, -1.0f});
        REQUIRE(particle2.GetPosition() == math::Point3{0, 0, 7.0f});
    }

    SECTION("Resolve interpenetration when touching")
    {
        physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(4.0f);
        physics::ParticleContact contact(&particle1, &particle2, 0.5f, 0.0f);
        contact.Resolve(1.0f);
        REQUIRE(particle1.GetPosition() == math::Point3{0, 0, 0.0f});
        REQUIRE(particle2.GetPosition() == math::Point3{0, 0, 5.0f});
    }

    SECTION("Resolve interpenetration when not touching")
    {
        physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(4.0f);
        physics::ParticleContact contact(&particle1, &particle2, 0.5f, -1.0f);
        contact.Resolve(1.0f);
        REQUIRE(particle1.GetPosition() == math::Point3{0, 0, 0.0f});
        REQUIRE(particle2.GetPosition() == math::Point3{0, 0, 5.0f});
    }
}

TEST_CASE("Check particle resolver", "[particle][contact]")
{
    SECTION("Resolve contact")
    {
        physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        particle1.SetVelocity(math::Vector3{0, 0, 1});
        physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(4.0f);
        particle2.SetVelocity(math::Vector3{0, 0, -1});
        physics::ParticleContact contact(&particle1, &particle2, 0.5f, 1.0f);
        physics::ParticleContactResolver resolver(1);
        physics::ParticleContact contacts[] = {contact};
        resolver.ResolveContacts(contacts, 1.0f);
        REQUIRE(particle1.GetVelocity() == math::Vector3{0, 0, 0.0f});
        REQUIRE(particle2.GetVelocity() == math::Vector3{0, 0, 1.0f});
    }
    SECTION("Choose smallest separating velocity")
    {
        physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        particle1.SetVelocity(math::Vector3{0, 0, 1});
        physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(4.0f);
        particle2.SetVelocity(math::Vector3{0, 0, -1});
        physics::Particle particle3;
        particle3.SetPosition(math::Point3{0, 0, -1});
        particle3.SetInverseMass(2.0f);
        particle3.SetVelocity(math::Vector3{0, 0, PHYSICS_REALC(0.5)});
        physics::ParticleContact contact1(&particle1, &particle2, 0.5f, 1.0f);
        physics::ParticleContact contact2(&particle3, &particle2, 0.5f, 1.0f);
        physics::ParticleContact contacts[] = {contact1, contact2};
        physics::ParticleContactResolver resolver(1);
        REQUIRE(resolver.GetIterations() == 1);
        resolver.ResolveContacts(contacts, 1.0f);
        REQUIRE(resolver.GetIterationsUsed() == 1);
        REQUIRE(particle1.GetVelocity() == math::Vector3{0, 0, 0.0f});
        REQUIRE(particle2.GetVelocity() == math::Vector3{0, 0, 1.0f});

        resolver.SetIterations(3);
        REQUIRE(resolver.GetIterations() == 3);
    }
}
