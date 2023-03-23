#include <catch2/catch_test_macros.hpp>

#include "physics/particle-links.h"
#include "physics/particle.h"

TEST_CASE("Cable", "ParticleContact")
{
    physics::Particle Particle1;
    Particle1.SetInverseMass(2.0f);
    Particle1.SetVelocity(math::Vector3{0, 0, 1});
    physics::Particle Particle2;
    Particle2.SetPosition(math::Point3{0, 0, 5});
    Particle2.SetInverseMass(4.0f);
    Particle2.SetVelocity(math::Vector3{0, 0, -1});

    SECTION("Generate contact for stretched cable")
    {
        physics::ParticleCable Cable(&Particle1, &Particle2, 4.0f, 0.5f);
        physics::StackArray<physics::ParticleContact, 1> Contacts;
        const uint32_t AddedContactsCount = Cable.AddContact(Contacts);
        REQUIRE(AddedContactsCount == 1);
        REQUIRE(Contacts[0].GetMainParticle() == &Particle1);
        REQUIRE(Contacts[0].GetOtherParticle() == &Particle2);
        REQUIRE(Contacts[0].GetContactNormal() == math::Vector3{0, 0, -1});
        REQUIRE(Contacts[0].GetPenetration() == 1.0f);
        REQUIRE(Contacts[0].GetRestitution() == 0.5f);
    }

    SECTION("Generate contact for stretched cable")
    {
        physics::ParticleCable Cable(&Particle1, &Particle2, 4.0f, 0.5f);
        physics::StackArray<physics::ParticleContact, 1> Contacts;
        const uint32_t AddedContactsCount = Cable.AddContact(Contacts);
        REQUIRE(AddedContactsCount == 1);
        REQUIRE(Contacts[0].IsValid());
        REQUIRE(Contacts[0].GetMainParticle() == &Particle1);
        REQUIRE(Contacts[0].GetOtherParticle() == &Particle2);
        REQUIRE(Contacts[0].GetContactNormal() == math::Vector3{0, 0, -1});
        REQUIRE(Contacts[0].GetPenetration() == 1.0f);
        REQUIRE(Contacts[0].GetRestitution() == 0.5f);
    }
    SECTION("Generate contact for relaxed cable")
    {
        Particle2.SetPosition(math::Point3{0, 0, 3});
        physics::ParticleCable Cable(&Particle1, &Particle2, 4.0f, 0.5f);
        physics::StackArray<physics::ParticleContact, 1> Contacts;
        const uint32_t AddedContactsCount = Cable.AddContact(Contacts);
        REQUIRE(AddedContactsCount == 0);
        REQUIRE(!Contacts[0].IsValid());
        REQUIRE(Contacts[0].GetMainParticle() == nullptr);
        REQUIRE(Contacts[0].GetOtherParticle() == nullptr);
        REQUIRE(Contacts[0].GetContactNormal() == math::Vector3::Zero);
        REQUIRE(Contacts[0].GetPenetration() == 0.0f);
        REQUIRE(Contacts[0].GetRestitution() == 0.0f);
    }
    SECTION("Generate contact for max length cable")
    {
        Particle2.SetPosition(math::Point3{0, 0, 4});
        physics::ParticleCable Cable(&Particle1, &Particle2, 4.0f, 0.5f);
        physics::StackArray<physics::ParticleContact, 1> Contacts;
        const uint32_t AddedContactsCount = Cable.AddContact(Contacts);
        REQUIRE(AddedContactsCount == 1);
        REQUIRE(Contacts[0].IsValid());
        REQUIRE(Contacts[0].GetMainParticle() == &Particle1);
        REQUIRE(Contacts[0].GetOtherParticle() == &Particle2);
        REQUIRE(Contacts[0].GetContactNormal() == math::Vector3{0, 0, -1});
        REQUIRE(Contacts[0].GetPenetration() == 0.0f);
        REQUIRE(Contacts[0].GetRestitution() == 0.5f);
    }
    SECTION("Setters")
    {
        physics::ParticleCable Cable(&Particle1, &Particle2, 4.0f, 0.5f);
        REQUIRE(Cable.GetMaxLength() == 4.0f);
        REQUIRE(Cable.GetRestitution() == 0.5f);
        REQUIRE(Cable.GetFirstParticle() == &Particle1);
        REQUIRE(Cable.GetSecondParticle() == &Particle2);
        Cable.SetMaxLength(5.0f);
        Cable.SetRestitution(0.6f);
        REQUIRE(Cable.GetMaxLength() == 5.0f);
        REQUIRE(Cable.GetRestitution() == 0.6f);
        Cable.SetFirstParticle(nullptr);
        Cable.SetSecondParticle(nullptr);
        REQUIRE(Cable.GetFirstParticle() == nullptr);
        REQUIRE(Cable.GetSecondParticle() == nullptr);
    }
}

TEST_CASE("Rod", "Particle")
{
    physics::Particle Particle1;
    Particle1.SetInverseMass(2.0f);
    physics::Particle Particle2;
    Particle2.SetPosition(math::Point3{0, 0, 5});
    Particle2.SetInverseMass(4.0f);

    SECTION("Generate contact for stretched rod")
    {
        physics::ParticleRod Rod(&Particle1, &Particle2, 4.0f);
        physics::StackArray<physics::ParticleContact, 1> Contacts;
        const uint32_t AddedContactsCount = Rod.AddContact(Contacts);
        REQUIRE(AddedContactsCount == 1);
        REQUIRE(Contacts[0].GetMainParticle() == &Particle1);
        REQUIRE(Contacts[0].GetOtherParticle() == &Particle2);
        REQUIRE(Contacts[0].GetContactNormal() == math::Vector3{0, 0, 1});
        REQUIRE(Contacts[0].GetPenetration() == 1.0f);
        REQUIRE(Contacts[0].GetRestitution() == 0.0f);
    }
    SECTION("Generate contact for relaxed rod")
    {
        Particle2.SetPosition(math::Point3{0, 0, 3});
        physics::ParticleRod Rod(&Particle1, &Particle2, 4.0f);
        physics::StackArray<physics::ParticleContact, 1> Contacts;
        const uint32_t AddedContactsCount = Rod.AddContact(Contacts);
        REQUIRE(AddedContactsCount == 1);
        REQUIRE(Contacts[0].IsValid());
        REQUIRE(Contacts[0].GetMainParticle() == &Particle1);
        REQUIRE(Contacts[0].GetOtherParticle() == &Particle2);
        REQUIRE(Contacts[0].GetContactNormal() == math::Vector3(0, 0, -1));
        REQUIRE(Contacts[0].GetPenetration() == 1.0f);
        REQUIRE(Contacts[0].GetRestitution() == 0.0f);
    }
    SECTION("Generate contact for max length rod")
    {
        Particle2.SetPosition(math::Point3{0, 0, 4});
        physics::ParticleRod Rod(&Particle1, &Particle2, 4.0f);
        physics::StackArray<physics::ParticleContact, 1> Contacts;
        const uint32_t AddedContactsCount = Rod.AddContact(Contacts);
        REQUIRE(AddedContactsCount == 0);
        REQUIRE(!Contacts[0].IsValid());
        REQUIRE(Contacts[0].GetMainParticle() == nullptr);
        REQUIRE(Contacts[0].GetOtherParticle() == nullptr);
        REQUIRE(Contacts[0].GetContactNormal() == math::Vector3::Zero);
        REQUIRE(Contacts[0].GetPenetration() == 0.0f);
        REQUIRE(Contacts[0].GetRestitution() == 0.0f);
    }
}
