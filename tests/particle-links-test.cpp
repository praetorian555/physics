#include <catch2/catch_test_macros.hpp>

#include "physics/particle-links.h"
#include "physics/particle.h"

TEST_CASE("Particle cable link", "[particle][link]")
{
    physics::Particle particle1;
    particle1.SetInverseMass(2.0f);
    particle1.SetVelocity(math::Vector3{0, 0, 1});
    physics::Particle particle2;
    particle2.SetPosition(math::Point3{0, 0, 5});
    particle2.SetInverseMass(4.0f);
    particle2.SetVelocity(math::Vector3{0, 0, -1});

    SECTION("Generate contact for stretched cable")
    {
        physics::ParticleCable cable(&particle1, &particle2, 4.0f, 0.5f);
        physics::StackArray<physics::ParticleContact, 1> contacts;
        const uint32_t added_contacts_count = cable.AddContact(contacts);
        REQUIRE(added_contacts_count == 1);
        REQUIRE(contacts[0].GetMainParticle() == &particle1);
        REQUIRE(contacts[0].GetOtherParticle() == &particle2);
        REQUIRE(contacts[0].GetContactNormal() == math::Vector3{0, 0, -1});
        REQUIRE(contacts[0].GetPenetration() == 1.0f);
        REQUIRE(contacts[0].GetRestitution() == 0.5f);
    }

    SECTION("Generate contact for stretched cable")
    {
        physics::ParticleCable cable(&particle1, &particle2, 4.0f, 0.5f);
        physics::StackArray<physics::ParticleContact, 1> contacts;
        const uint32_t added_contacts_count = cable.AddContact(contacts);
        REQUIRE(added_contacts_count == 1);
        REQUIRE(contacts[0].IsValid());
        REQUIRE(contacts[0].GetMainParticle() == &particle1);
        REQUIRE(contacts[0].GetOtherParticle() == &particle2);
        REQUIRE(contacts[0].GetContactNormal() == math::Vector3{0, 0, -1});
        REQUIRE(contacts[0].GetPenetration() == 1.0f);
        REQUIRE(contacts[0].GetRestitution() == 0.5f);
    }
    SECTION("Generate contact for relaxed cable")
    {
        particle2.SetPosition(math::Point3{0, 0, 3});
        physics::ParticleCable cable(&particle1, &particle2, 4.0f, 0.5f);
        physics::StackArray<physics::ParticleContact, 1> contacts;
        const uint32_t added_contacts_count = cable.AddContact(contacts);
        REQUIRE(added_contacts_count == 0);
        REQUIRE(!contacts[0].IsValid());
        REQUIRE(contacts[0].GetMainParticle() == nullptr);
        REQUIRE(contacts[0].GetOtherParticle() == nullptr);
        REQUIRE(contacts[0].GetContactNormal() == math::Vector3::Zero);
        REQUIRE(contacts[0].GetPenetration() == 0.0f);
        REQUIRE(contacts[0].GetRestitution() == 0.0f);
    }
    SECTION("Generate contact for max length cable")
    {
        particle2.SetPosition(math::Point3{0, 0, 4});
        physics::ParticleCable cable(&particle1, &particle2, 4.0f, 0.5f);
        physics::StackArray<physics::ParticleContact, 1> contacts;
        const uint32_t added_contacts_count = cable.AddContact(contacts);
        REQUIRE(added_contacts_count == 1);
        REQUIRE(contacts[0].IsValid());
        REQUIRE(contacts[0].GetMainParticle() == &particle1);
        REQUIRE(contacts[0].GetOtherParticle() == &particle2);
        REQUIRE(contacts[0].GetContactNormal() == math::Vector3{0, 0, -1});
        REQUIRE(contacts[0].GetPenetration() == 0.0f);
        REQUIRE(contacts[0].GetRestitution() == 0.5f);
    }
    SECTION("Setters")
    {
        physics::ParticleCable cable(&particle1, &particle2, 4.0f, 0.5f);
        REQUIRE(cable.GetMaxLength() == 4.0f);
        REQUIRE(cable.GetRestitution() == 0.5f);
        REQUIRE(cable.GetFirstParticle() == &particle1);
        REQUIRE(cable.GetSecondParticle() == &particle2);
        cable.SetMaxLength(5.0f);
        cable.SetRestitution(0.6f);
        REQUIRE(cable.GetMaxLength() == 5.0f);
        REQUIRE(cable.GetRestitution() == 0.6f);
        cable.SetFirstParticle(nullptr);
        cable.SetSecondParticle(nullptr);
        REQUIRE(cable.GetFirstParticle() == nullptr);
        REQUIRE(cable.GetSecondParticle() == nullptr);
    }
}

TEST_CASE("Particle rod link", "[particle][link]")
{
    physics::Particle particle1;
    particle1.SetInverseMass(2.0f);
    physics::Particle particle2;
    particle2.SetPosition(math::Point3{0, 0, 5});
    particle2.SetInverseMass(4.0f);

    SECTION("Generate contact for stretched rod")
    {
        physics::ParticleRod rod(&particle1, &particle2, 4.0f);
        physics::StackArray<physics::ParticleContact, 1> contacts;
        const uint32_t added_contacts_count = rod.AddContact(contacts);
        REQUIRE(added_contacts_count == 1);
        REQUIRE(contacts[0].GetMainParticle() == &particle1);
        REQUIRE(contacts[0].GetOtherParticle() == &particle2);
        REQUIRE(contacts[0].GetContactNormal() == math::Vector3{0, 0, 1});
        REQUIRE(contacts[0].GetPenetration() == 1.0f);
        REQUIRE(contacts[0].GetRestitution() == 0.0f);
    }
    SECTION("Generate contact for relaxed rod")
    {
        particle2.SetPosition(math::Point3{0, 0, 3});
        physics::ParticleRod rod(&particle1, &particle2, 4.0f);
        physics::StackArray<physics::ParticleContact, 1> contacts;
        const uint32_t added_contacts_count = rod.AddContact(contacts);
        REQUIRE(added_contacts_count == 1);
        REQUIRE(contacts[0].IsValid());
        REQUIRE(contacts[0].GetMainParticle() == &particle1);
        REQUIRE(contacts[0].GetOtherParticle() == &particle2);
        REQUIRE(contacts[0].GetContactNormal() == math::Vector3(0, 0, -1));
        REQUIRE(contacts[0].GetPenetration() == 1.0f);
        REQUIRE(contacts[0].GetRestitution() == 0.0f);
    }
    SECTION("Generate contact for max length rod")
    {
        particle2.SetPosition(math::Point3{0, 0, 4});
        physics::ParticleRod rod(&particle1, &particle2, 4.0f);
        physics::StackArray<physics::ParticleContact, 1> contacts;
        const uint32_t added_contacts_count = rod.AddContact(contacts);
        REQUIRE(added_contacts_count == 0);
        REQUIRE(!contacts[0].IsValid());
        REQUIRE(contacts[0].GetMainParticle() == nullptr);
        REQUIRE(contacts[0].GetOtherParticle() == nullptr);
        REQUIRE(contacts[0].GetContactNormal() == math::Vector3::Zero);
        REQUIRE(contacts[0].GetPenetration() == 0.0f);
        REQUIRE(contacts[0].GetRestitution() == 0.0f);
    }
}
