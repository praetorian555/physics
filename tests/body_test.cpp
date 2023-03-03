#include <catch2/catch_test_macros.hpp>

#include "physics/body.h"
#include "physics/forcegenerator.h"

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

    Body.SetAngularDamping(10);
    REQUIRE(Body.GetAngularDamping() == 10);

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

    math::Vector3 Acceleration = {1, 2, 3};
    Body.SetAcceleration(Acceleration);
    REQUIRE(Body.GetAcceleration() == Acceleration);
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
    REQUIRE(Body.GetInverseInertiaTensorWorld() ==
            math::Rotate(Orientation) * InverseInertiaTensor);
}

TEST_CASE("AddForce", "Body")
{
    {
        physics::RigidBody Body;
        const math::Vector3 Force = {1, 2, 3};
        Body.AddForce(Force);
        REQUIRE(Body.GetAccumulatedForce() == Force);
        REQUIRE(Body.GetAccumulatedTorque() == math::Vector3::Zero);
        Body.AddForce(Force);
        REQUIRE(Body.GetAccumulatedForce() == Force * 2);
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

TEST_CASE("BodyGravity", "Body")
{
    physics::RigidBody Body;
    Body.SetMass(5);
    physics::Gravity Gravity{math::Vector3{0, -PHYSICS_REALC(10.0), 0}};
    Gravity.UpdateForce(Body, 1);
    REQUIRE(Body.GetAccumulatedForce() == math::Vector3{0, -PHYSICS_REALC(50.0), 0});
    REQUIRE(Body.GetAccumulatedTorque() == math::Vector3::Zero);

    Gravity.SetGravity({0, PHYSICS_REALC(5.0), 0});
    REQUIRE(Gravity.GetGravity() == math::Vector3{0, PHYSICS_REALC(5.0), 0});
}

TEST_CASE("BodySpring", "Body")
{
    {
        physics::RigidBody OtherBody;
        const math::Point3 ConnectionPoint = {1, 2, 3};
        const math::Point3 OtherConnectionPoint = {4, 5, 6};
        physics::Spring S(ConnectionPoint, &OtherBody, OtherConnectionPoint, 5, 10);
        REQUIRE(S.GetConnectionPointLocal() == ConnectionPoint);
        REQUIRE(S.GetOtherConnectionPointLocal() == OtherConnectionPoint);
        REQUIRE(S.GetOtherBody() == &OtherBody);
        REQUIRE(S.GetSpringConstant() == 5);
        REQUIRE(S.GetRestLength() == 10);

        S.SetRestLength(20);
        REQUIRE(S.GetRestLength() == 20);
        S.SetSpringConstant(10);
        REQUIRE(S.GetSpringConstant() == 10);
        S.SetConnectionPointLocal({0, 0, 0});
        REQUIRE(S.GetConnectionPointLocal() == math::Point3::Zero);
        S.SetOtherConnectionPointLocal({0, 0, 0});
        REQUIRE(S.GetOtherConnectionPointLocal() == math::Point3::Zero);
        S.SetOtherBody(nullptr);
        REQUIRE(S.GetOtherBody() == nullptr);
    }
    {
        physics::RigidBody Body;
        Body.SetPosition({1, 2, 3});
        Body.CalculateDerivedData();
        physics::RigidBody OtherBody;
        OtherBody.SetPosition({1, 5, 7});
        OtherBody.CalculateDerivedData();
        const math::Point3 ConnectionPoint = {0, 0, 0};
        physics::Spring S(ConnectionPoint, &OtherBody, ConnectionPoint, 5, 10);
        S.UpdateForce(Body, 1);
        REQUIRE(Body.GetAccumulatedForce().X == 0);
        REQUIRE(Body.GetAccumulatedForce().Z == -20);

#if PHYSICS_REAL_AS_DOUBLE
        constexpr physics::real kEpsilon = PHYSICS_REALC(1e-6);
#else
        constexpr physics::real kEpsilon = PHYSICS_REALC(1e-3);
#endif
        REQUIRE(math::IsEqual(Body.GetAccumulatedForce().Y, -15, kEpsilon));
    }
}

TEST_CASE("BodyIntegration", "Body")
{
    {
        physics::RigidBody Body;
        Body.SetMass(2);
        Body.SetDamping(0.5);
        Body.SetPosition({1, 2, 3});
        Body.SetVelocity({1, 2, 3});
        Body.AddForce({1, 2, 3});
        Body.Integrate(1);
        REQUIRE(Body.GetVelocity() ==
                math::Vector3{PHYSICS_REALC(0.75), PHYSICS_REALC(1.5), PHYSICS_REALC(2.25)});
        REQUIRE(Body.GetPosition() ==
                math::Point3{PHYSICS_REALC(1.75), PHYSICS_REALC(3.5), PHYSICS_REALC(5.25)});
        REQUIRE(Body.GetAccumulatedForce() == math::Vector3::Zero);
        REQUIRE(Body.GetTransform() == math::Translate(Body.GetPosition()));
    }
    {
        physics::RigidBody Body;
        Body.SetMass(2);
        Body.SetDamping(0.5);
        Body.SetPosition({1, 2, 3});
        Body.SetVelocity({1, 2, 3});
        Body.AddForceAtPoint({1, 2, 3}, {1, 1, 1});
        Body.Integrate(1);
        REQUIRE(Body.GetVelocity() ==
                math::Vector3{PHYSICS_REALC(0.75), PHYSICS_REALC(1.5), PHYSICS_REALC(2.25)});
        REQUIRE(Body.GetPosition() ==
                math::Point3{PHYSICS_REALC(1.75), PHYSICS_REALC(3.5), PHYSICS_REALC(5.25)});
        REQUIRE(Body.GetAccumulatedForce() == math::Vector3::Zero);
    }
    {
        physics::RigidBody Body;
        Body.SetMass(2);
        Body.SetDamping(0.5);
        Body.SetPosition({1, 2, 3});
        Body.SetVelocity({1, 2, 3});
        Body.AddForceAtLocalPoint({1, 2, 3}, {1, 1, 1});
        Body.Integrate(1);
        REQUIRE(Body.GetVelocity() ==
                math::Vector3{PHYSICS_REALC(0.75), PHYSICS_REALC(1.5), PHYSICS_REALC(2.25)});
        REQUIRE(Body.GetPosition() ==
                math::Point3{PHYSICS_REALC(1.75), PHYSICS_REALC(3.5), PHYSICS_REALC(5.25)});
        REQUIRE(Body.GetAccumulatedForce() == math::Vector3::Zero);
    }
    {
        physics::RigidBody Body;
        Body.SetInverseInertiaTensor(math::Scale(3));
        Body.CalculateDerivedData();
        Body.SetAngularDamping(0.5);
        Body.SetOrientation(math::Quaternion::FromAxisAngleDegrees({0, 0, 1}, 90));
        Body.AddForceAtPoint({0, 1, 0}, {1, 0, 0});
        REQUIRE(Body.GetAccumulatedTorque() == math::Vector3{0, 0, 1});
        Body.Integrate(1);
        REQUIRE(Body.GetAngularVelocity() == math::Vector3{0, 0, PHYSICS_REALC(1.5)});
        REQUIRE(Body.GetAccumulatedTorque() == math::Vector3::Zero);

        const math::Quaternion Orientation = Body.GetOrientation();
        const math::Vector3 Axis = math::Normalize(Orientation.Vec);
        physics::real Angle = std::acos(Orientation.W) * 2;
        Angle = math::Degrees(Angle);
        REQUIRE(Axis == math::Vector3{0, 0, 1});
        REQUIRE(Angle > 90);
    }
}
