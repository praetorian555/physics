#include <catch2/catch_test_macros.hpp>

#include "physics/particle-contacts.h"
#include "physics/particle.h"

TEST_CASE("Creation of particle contact", "[creation][particle][contact]")
{
    SECTION("Create with two particles")
    {
        Physics::Particle particle1;
        Physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 5, 3});
        Physics::ParticleContact contact(&particle1, &particle2, 0.5f, 1.0f);
        REQUIRE(contact.IsValid());
        REQUIRE(contact.GetMainParticle() == &particle1);
        REQUIRE(contact.GetOtherParticle() == &particle2);
        REQUIRE(contact.GetRestitution() == 0.5f);
        REQUIRE(contact.GetPenetration() == 1.0f);
        REQUIRE(contact.GetContactNormal() ==
                math::Normalize(particle1.GetPosition() - particle2.GetPosition()));

        contact = Physics::ParticleContact(&particle2, nullptr, 0.5f, 1.0f);
        REQUIRE(!contact.IsValid());
        contact = Physics::ParticleContact(nullptr, &particle2, 0.5f, 1.0f);
        REQUIRE(!contact.IsValid());
        contact = Physics::ParticleContact(&particle1, &particle2, -0.5f, 1.0f);
        REQUIRE(!contact.IsValid());
        contact = Physics::ParticleContact(&particle1, &particle2, 1.5f, 1.0f);
        REQUIRE(!contact.IsValid());
    }

    SECTION("Create with one particle")
    {
        Physics::Particle particle1;
        Physics::ParticleContact contact(&particle1, 0.5f, 2.0f, math::Vector3{0, 0, 3});
        REQUIRE(contact.IsValid());
        REQUIRE(contact.GetMainParticle() == &particle1);
        REQUIRE(contact.GetOtherParticle() == nullptr);
        REQUIRE(contact.GetRestitution() == 0.5f);
        REQUIRE(contact.GetPenetration() == 2.0f);
        REQUIRE(contact.GetContactNormal() == math::Normalize(math::Vector3{0, 0, 1}));

        contact = Physics::ParticleContact(nullptr, 0.5f, 1.0f, math::Vector3{0, 0, 3});
        REQUIRE(!contact.IsValid());
        contact = Physics::ParticleContact(&particle1, -0.5f, 1.0f, math::Vector3{0, 0, 3});
        REQUIRE(!contact.IsValid());
        contact = Physics::ParticleContact(&particle1, 1.5f, 1.0f, math::Vector3{0, 0, 3});
        REQUIRE(!contact.IsValid());
        contact = Physics::ParticleContact(&particle1, 0.5f, 1.0f, math::Vector3{0, 0, 0});
        REQUIRE(!contact.IsValid());
    }
}

TEST_CASE("Resolving a contact", "[particle][contact]")
{
    SECTION("Resolve velocity when approaching")
    {
        Physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        particle1.SetVelocity(math::Vector3{0, 0, 1});
        Physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(4.0f);
        particle2.SetVelocity(math::Vector3{0, 0, -1});
        Physics::ParticleContact contact(&particle1, &particle2, 0.5f, 1.0f);
        contact.Resolve(1.0f);
        REQUIRE(particle1.GetVelocity() == math::Vector3{0, 0, 0.0f});
        REQUIRE(particle2.GetVelocity() == math::Vector3{0, 0, 1.0f});
    }
    SECTION("Resolve velocity when separating")
    {
        Physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        particle1.SetVelocity(math::Vector3{0, 0, -1});
        Physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(4.0f);
        particle2.SetVelocity(math::Vector3{0, 0, 1});
        Physics::ParticleContact contact(&particle1, &particle2, 0.5f, 1.0f);
        contact.Resolve(1.0f);
        REQUIRE(particle1.GetVelocity() == math::Vector3{0, 0, -1.0f});
        REQUIRE(particle2.GetVelocity() == math::Vector3{0, 0, 1.0f});
    }
    SECTION("Resolve velocity when immovable objects")
    {
        Physics::Particle particle1;
        particle1.SetInverseMass(0.0f);
        particle1.SetVelocity(math::Vector3{0, 0, -1});
        Physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(0.0f);
        particle2.SetVelocity(math::Vector3{0, 0, 1});
        Physics::ParticleContact contact(&particle1, &particle2, 0.5f, 1.0f);
        contact.Resolve(1.0f);
        REQUIRE(particle1.GetVelocity() == math::Vector3{0, 0, -1.0f});
        REQUIRE(particle2.GetVelocity() == math::Vector3{0, 0, 1.0f});
    }

    SECTION("Resolve velocity with rest contact")
    {
        Physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        particle1.SetVelocity(math::Vector3{0, 0, -1});
        particle1.SetAcceleration(math::Vector3{0, 0, -1});
        Physics::ParticleContact contact(&particle1, 1.0f, 1.0f, math::Vector3{0, 0, 1});
        contact.Resolve(1.0f);
        REQUIRE(particle1.GetVelocity() == math::Vector3{0, 0, 0.0f});
    }

    SECTION("Resolve interpenetration")
    {
        Physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        Physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(4.0f);
        Physics::ParticleContact contact(&particle1, &particle2, 0.5f, 3.0f);
        contact.Resolve(1.0f);
        REQUIRE(particle1.GetPosition() == math::Point3{0, 0, -1.0f});
        REQUIRE(particle2.GetPosition() == math::Point3{0, 0, 7.0f});
    }

    SECTION("Resolve interpenetration when touching")
    {
        Physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        Physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(4.0f);
        Physics::ParticleContact contact(&particle1, &particle2, 0.5f, 0.0f);
        contact.Resolve(1.0f);
        REQUIRE(particle1.GetPosition() == math::Point3{0, 0, 0.0f});
        REQUIRE(particle2.GetPosition() == math::Point3{0, 0, 5.0f});
    }

    SECTION("Resolve interpenetration when not touching")
    {
        Physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        Physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(4.0f);
        Physics::ParticleContact contact(&particle1, &particle2, 0.5f, -1.0f);
        contact.Resolve(1.0f);
        REQUIRE(particle1.GetPosition() == math::Point3{0, 0, 0.0f});
        REQUIRE(particle2.GetPosition() == math::Point3{0, 0, 5.0f});
    }
}

TEST_CASE("Check particle resolver", "[particle][contact]")
{
    SECTION("Resolve contact")
    {
        Physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        particle1.SetVelocity(math::Vector3{0, 0, 1});
        Physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(4.0f);
        particle2.SetVelocity(math::Vector3{0, 0, -1});
        Physics::ParticleContact contact(&particle1, &particle2, 0.5f, 1.0f);
        Physics::ParticleContactResolver resolver(1);
        Physics::ParticleContact contacts[] = {contact};
        resolver.ResolveContacts(contacts, 1.0f);
        REQUIRE(particle1.GetVelocity() == math::Vector3{0, 0, 0.0f});
        REQUIRE(particle2.GetVelocity() == math::Vector3{0, 0, 1.0f});
    }
    SECTION("Choose smallest separating velocity")
    {
        Physics::Particle particle1;
        particle1.SetInverseMass(2.0f);
        particle1.SetVelocity(math::Vector3{0, 0, 1});
        Physics::Particle particle2;
        particle2.SetPosition(math::Point3{0, 0, 5});
        particle2.SetInverseMass(4.0f);
        particle2.SetVelocity(math::Vector3{0, 0, -1});
        Physics::Particle particle3;
        particle3.SetPosition(math::Point3{0, 0, -1});
        particle3.SetInverseMass(2.0f);
        particle3.SetVelocity(math::Vector3{0, 0, PHYSICS_REALC(0.5)});
        Physics::ParticleContact contact1(&particle1, &particle2, 0.5f, 1.0f);
        Physics::ParticleContact contact2(&particle3, &particle2, 0.5f, 1.0f);
        Physics::ParticleContact contacts[] = {contact1, contact2};
        Physics::ParticleContactResolver resolver(1);
        REQUIRE(resolver.GetIterations() == 1);
        resolver.ResolveContacts(contacts, 1.0f);
        REQUIRE(resolver.GetIterationsUsed() == 1);
        REQUIRE(particle1.GetVelocity() == math::Vector3{0, 0, 0.0f});
        REQUIRE(particle2.GetVelocity() == math::Vector3{0, 0, 1.0f});

        resolver.SetIterations(3);
        REQUIRE(resolver.GetIterations() == 3);
    }
}
