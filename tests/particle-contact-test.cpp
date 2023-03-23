#include <catch2/catch_test_macros.hpp>

#include "physics/particle-contacts.h"
#include "physics/particle.h"

TEST_CASE("Creation", "ParticleContact")
{
    SECTION("Create with two particles")
    {
        physics::Particle Particle1;
        physics::Particle Particle2;
        Particle2.SetPosition(math::Point3{0, 5, 3});
        physics::ParticleContact Contact(&Particle1, &Particle2, 0.5f, 1.0f);
        REQUIRE(Contact.IsValid());
        REQUIRE(Contact.GetMainParticle() == &Particle1);
        REQUIRE(Contact.GetOtherParticle() == &Particle2);
        REQUIRE(Contact.GetRestitution() == 0.5f);
        REQUIRE(Contact.GetPenetration() == 1.0f);
        REQUIRE(Contact.GetContactNormal() ==
                math::Normalize(Particle1.GetPosition() - Particle2.GetPosition()));

        Contact = physics::ParticleContact(&Particle2, nullptr, 0.5f, 1.0f);
        REQUIRE(!Contact.IsValid());
        Contact = physics::ParticleContact(nullptr, &Particle2, 0.5f, 1.0f);
        REQUIRE(!Contact.IsValid());
        Contact = physics::ParticleContact(&Particle1, &Particle2, -0.5f, 1.0f);
        REQUIRE(!Contact.IsValid());
        Contact = physics::ParticleContact(&Particle1, &Particle2, 1.5f, 1.0f);
        REQUIRE(!Contact.IsValid());
    }

    SECTION("Create with one particle")
    {
        physics::Particle Particle1;
        physics::ParticleContact Contact(&Particle1, 0.5f, 2.0f, math::Vector3{0, 0, 3});
        REQUIRE(Contact.IsValid());
        REQUIRE(Contact.GetMainParticle() == &Particle1);
        REQUIRE(Contact.GetOtherParticle() == nullptr);
        REQUIRE(Contact.GetRestitution() == 0.5f);
        REQUIRE(Contact.GetPenetration() == 2.0f);
        REQUIRE(Contact.GetContactNormal() == math::Normalize(math::Vector3{0, 0, 1}));

        Contact = physics::ParticleContact(nullptr, 0.5f, 1.0f, math::Vector3{0, 0, 3});
        REQUIRE(!Contact.IsValid());
        Contact = physics::ParticleContact(&Particle1, -0.5f, 1.0f, math::Vector3{0, 0, 3});
        REQUIRE(!Contact.IsValid());
        Contact = physics::ParticleContact(&Particle1, 1.5f, 1.0f, math::Vector3{0, 0, 3});
        REQUIRE(!Contact.IsValid());
        Contact = physics::ParticleContact(&Particle1, 0.5f, 1.0f, math::Vector3{0, 0, 0});
        REQUIRE(!Contact.IsValid());
    }
}

TEST_CASE("Resolve", "ParticleContact")
{
    SECTION("Resolve velocity when approaching")
    {
        physics::Particle Particle1;
        Particle1.SetInverseMass(2.0f);
        Particle1.SetVelocity(math::Vector3{0, 0, 1});
        physics::Particle Particle2;
        Particle2.SetPosition(math::Point3{0, 0, 5});
        Particle2.SetInverseMass(4.0f);
        Particle2.SetVelocity(math::Vector3{0, 0, -1});
        physics::ParticleContact Contact(&Particle1, &Particle2, 0.5f, 1.0f);
        Contact.Resolve(1.0f);
        REQUIRE(Particle1.GetVelocity() == math::Vector3{0, 0, 0.0f});
        REQUIRE(Particle2.GetVelocity() == math::Vector3{0, 0, 1.0f});
    }
    SECTION("Resolve velocity when separating")
    {
        physics::Particle Particle1;
        Particle1.SetInverseMass(2.0f);
        Particle1.SetVelocity(math::Vector3{0, 0, -1});
        physics::Particle Particle2;
        Particle2.SetPosition(math::Point3{0, 0, 5});
        Particle2.SetInverseMass(4.0f);
        Particle2.SetVelocity(math::Vector3{0, 0, 1});
        physics::ParticleContact Contact(&Particle1, &Particle2, 0.5f, 1.0f);
        Contact.Resolve(1.0f);
        REQUIRE(Particle1.GetVelocity() == math::Vector3{0, 0, -1.0f});
        REQUIRE(Particle2.GetVelocity() == math::Vector3{0, 0, 1.0f});
    }
    SECTION("Resolve velocity when immovable objects")
    {
        physics::Particle Particle1;
        Particle1.SetInverseMass(0.0f);
        Particle1.SetVelocity(math::Vector3{0, 0, -1});
        physics::Particle Particle2;
        Particle2.SetPosition(math::Point3{0, 0, 5});
        Particle2.SetInverseMass(0.0f);
        Particle2.SetVelocity(math::Vector3{0, 0, 1});
        physics::ParticleContact Contact(&Particle1, &Particle2, 0.5f, 1.0f);
        Contact.Resolve(1.0f);
        REQUIRE(Particle1.GetVelocity() == math::Vector3{0, 0, -1.0f});
        REQUIRE(Particle2.GetVelocity() == math::Vector3{0, 0, 1.0f});
    }

    SECTION("Resolve velocity with rest contact")
    {
        physics::Particle Particle1;
        Particle1.SetInverseMass(2.0f);
        Particle1.SetVelocity(math::Vector3{0, 0, -1});
        Particle1.SetAcceleration(math::Vector3{0, 0, -1});
        physics::ParticleContact Contact(&Particle1, 1.0f, 1.0f, math::Vector3{0, 0, 1});
        Contact.Resolve(1.0f);
        REQUIRE(Particle1.GetVelocity() == math::Vector3{0, 0, 0.0f});
    }

    SECTION("Resolve interpenetration")
    {
        physics::Particle Particle1;
        Particle1.SetInverseMass(2.0f);
        physics::Particle Particle2;
        Particle2.SetPosition(math::Point3{0, 0, 5});
        Particle2.SetInverseMass(4.0f);
        physics::ParticleContact Contact(&Particle1, &Particle2, 0.5f, 3.0f);
        Contact.Resolve(1.0f);
        REQUIRE(Particle1.GetPosition() == math::Point3{0, 0, -1.0f});
        REQUIRE(Particle2.GetPosition() == math::Point3{0, 0, 7.0f});
    }

    SECTION("Resolve interpenetration when touching")
    {
        physics::Particle Particle1;
        Particle1.SetInverseMass(2.0f);
        physics::Particle Particle2;
        Particle2.SetPosition(math::Point3{0, 0, 5});
        Particle2.SetInverseMass(4.0f);
        physics::ParticleContact Contact(&Particle1, &Particle2, 0.5f, 0.0f);
        Contact.Resolve(1.0f);
        REQUIRE(Particle1.GetPosition() == math::Point3{0, 0, 0.0f});
        REQUIRE(Particle2.GetPosition() == math::Point3{0, 0, 5.0f});
    }

    SECTION("Resolve interpenetration when not touching")
    {
        physics::Particle Particle1;
        Particle1.SetInverseMass(2.0f);
        physics::Particle Particle2;
        Particle2.SetPosition(math::Point3{0, 0, 5});
        Particle2.SetInverseMass(4.0f);
        physics::ParticleContact Contact(&Particle1, &Particle2, 0.5f, -1.0f);
        Contact.Resolve(1.0f);
        REQUIRE(Particle1.GetPosition() == math::Point3{0, 0, 0.0f});
        REQUIRE(Particle2.GetPosition() == math::Point3{0, 0, 5.0f});
    }
}

TEST_CASE("ParticleResolver", "ParticleContact")
{
    SECTION("Resolve contact")
    {
        physics::Particle Particle1;
        Particle1.SetInverseMass(2.0f);
        Particle1.SetVelocity(math::Vector3{0, 0, 1});
        physics::Particle Particle2;
        Particle2.SetPosition(math::Point3{0, 0, 5});
        Particle2.SetInverseMass(4.0f);
        Particle2.SetVelocity(math::Vector3{0, 0, -1});
        physics::ParticleContact Contact(&Particle1, &Particle2, 0.5f, 1.0f);
        physics::ParticleContactResolver Resolver(1);
        physics::ParticleContact Contacts[] = {Contact};
        Resolver.ResolveContacts(Contacts, 1.0f);
        REQUIRE(Particle1.GetVelocity() == math::Vector3{0, 0, 0.0f});
        REQUIRE(Particle2.GetVelocity() == math::Vector3{0, 0, 1.0f});
    }
    SECTION("Choose smallest separating velocity")
    {
        physics::Particle Particle1;
        Particle1.SetInverseMass(2.0f);
        Particle1.SetVelocity(math::Vector3{0, 0, 1});
        physics::Particle Particle2;
        Particle2.SetPosition(math::Point3{0, 0, 5});
        Particle2.SetInverseMass(4.0f);
        Particle2.SetVelocity(math::Vector3{0, 0, -1});
        physics::Particle Particle3;
        Particle3.SetPosition(math::Point3{0, 0, -1});
        Particle3.SetInverseMass(2.0f);
        Particle3.SetVelocity(math::Vector3{0, 0, PHYSICS_REALC(0.5)});
        physics::ParticleContact Contact1(&Particle1, &Particle2, 0.5f, 1.0f);
        physics::ParticleContact Contact2(&Particle3, &Particle2, 0.5f, 1.0f);
        physics::ParticleContact Contacts[] = {Contact1, Contact2};
        physics::ParticleContactResolver Resolver(1);
        REQUIRE(Resolver.GetIterations() == 1);
        Resolver.ResolveContacts(Contacts, 1.0f);
        REQUIRE(Resolver.GetIterationsUsed() == 1);
        REQUIRE(Particle1.GetVelocity() == math::Vector3{0, 0, 0.0f});
        REQUIRE(Particle2.GetVelocity() == math::Vector3{0, 0, 1.0f});

        Resolver.SetIterations(3);
        REQUIRE(Resolver.GetIterations() == 3);
    }
}
