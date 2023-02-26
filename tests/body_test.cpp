#include <catch2/catch_test_macros.hpp>

#include "physics/body.h"

TEST_CASE("GettersAndSetters", "Body")
{
    // Write tests for the getters and setters of the RigidBody class.
    physics::RigidBody Body;
    Body.SetMass(5);
    REQUIRE(Body.GetMass() == 5);
    REQUIRE(Body.GetInverseMass() == PHYSICS_REALC(0.2));
    Body.SetInverseMass(10);
    REQUIRE(Body.GetMass() == PHYSICS_REALC(0.1));
    REQUIRE(Body.GetInverseMass() == 10);
    REQUIRE(Body.HasFiniteMass() == true);

    math::Transform InertiaTensor = math::Scale(5);
    Body.SetInertiaTensor(InertiaTensor);
    REQUIRE(Body.GetInertiaTensor() == InertiaTensor);
    REQUIRE(Body.GetInverseInertiaTensor() == math::Scale(PHYSICS_REALC(0.2)));
    Body.SetInverseInertiaTensor(InertiaTensor);
    REQUIRE(Body.GetInertiaTensor() == math::Scale(PHYSICS_REALC(0.2)));
    REQUIRE(Body.GetInverseInertiaTensor() == InertiaTensor);

    Body.SetDamping(5);
    REQUIRE(Body.GetDamping() == 5);

    math::Point3 Position = {1, 2, 3};
    Body.SetPosition(Position);
    REQUIRE(Body.GetPosition() == Position);

    math::Quaternion Orientation = {1, 2, 3, 4};
    Body.SetOrientation(Orientation);
    REQUIRE(Body.GetOrientation() == Orientation);

    math::Vector3 Velocity = {1, 2, 3};
    Body.SetVelocity(Velocity);
    REQUIRE(Body.GetVelocity() == Velocity);

    math::Vector3 AngularVelocity = {1, 2, 3};
    Body.SetAngularVelocity(AngularVelocity);
    REQUIRE(Body.GetAngularVelocity() == AngularVelocity);
}

TEST_CASE("DerivedData", "Body")
{
    physics::RigidBody Body;
    const math::Point3 Position = {1, 2, 3};
    const math::Quaternion Orientation = math::Quaternion::FromAxisAngleDegrees({1, 0, 0}, 90);
    const math::Transform InverseInertiaTensor = math::Scale(5);
    Body.SetPosition(Position);
    Body.SetOrientation(Orientation);
    Body.SetInverseInertiaTensor(InverseInertiaTensor);
    REQUIRE(Body.GetTransform() == math::Transform{});
    REQUIRE(Body.GetInverseInertiaTensorWorld() == math::Transform{});
    Body.CalculateDerivedData();
    REQUIRE(Body.GetTransform() == math::Translate(Position) * math::Rotate(Orientation));
    REQUIRE(Body.GetInverseInertiaTensorWorld() == math::Rotate(Orientation) * InverseInertiaTensor);
}

TEST_CASE("AddForce", "Body")
{
    {
        physics::RigidBody Body;
        const math::Vector3 Force = {1, 2, 3};
        Body.AddForce(Force);
        REQUIRE(Body.GetAccumulatedForce() == Force);
        REQUIRE(Body.GetAccumulatedTorque() == math::Vector3::Zero);
        Body.ClearAccumulators();
        REQUIRE(Body.GetAccumulatedForce() == math::Vector3::Zero);
        REQUIRE(Body.GetAccumulatedTorque() == math::Vector3::Zero);
    }
    {
        physics::RigidBody Body;
        Body.SetPosition({1, 2, 3});
        const math::Vector3 Force = {1, 1, 1};
        Body.AddForceAtPoint(Force, {0, 0, 0});
        REQUIRE(Body.GetAccumulatedForce() == Force);
        REQUIRE(Body.GetAccumulatedTorque() == math::Vector3{1, -2, 1});
    }
    {
        physics::RigidBody Body;
        Body.SetPosition({1, 2, 3});
        Body.CalculateDerivedData();
        const math::Vector3 Force = {1, 1, 1};
        Body.AddForceAtLocalPoint(Force, {-1, -2, -3});
        REQUIRE(Body.GetAccumulatedForce() == Force);
        REQUIRE(Body.GetAccumulatedTorque() == math::Vector3{1, -2, 1});
    }
}
