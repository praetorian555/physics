#include <catch2/catch_test_macros.hpp>

#include "physics/body.h"
#include "physics/force-generator.h"

TEST_CASE("Creation of a body and getters and setters", "[creation][body]")
{
    Physics::RigidBody body;
    body.SetMass(5);
    REQUIRE(body.GetMass() == 5);
    REQUIRE(body.GetInverseMass() == PHYSICS_REALC(0.2));
    body.SetInverseMass(10);
    REQUIRE(body.GetMass() == PHYSICS_REALC(0.1));
    REQUIRE(body.GetInverseMass() == 10);
    REQUIRE(body.HasFiniteMass() == true);

    math::Transform inertia_tensor = math::Scale(5);
    body.SetInertiaTensor(inertia_tensor);
    REQUIRE(body.GetInertiaTensor() == inertia_tensor);
    REQUIRE(body.GetInverseInertiaTensor() == math::Scale(PHYSICS_REALC(0.2)));
    body.SetInverseInertiaTensor(inertia_tensor);
    REQUIRE(body.GetInertiaTensor() == math::Scale(PHYSICS_REALC(0.2)));
    REQUIRE(body.GetInverseInertiaTensor() == inertia_tensor);

    body.SetDamping(5);
    REQUIRE(body.GetDamping() == 5);

    body.SetAngularDamping(10);
    REQUIRE(body.GetAngularDamping() == 10);

    math::Point3 position = {1, 2, 3};
    body.SetPosition(position);
    REQUIRE(body.GetPosition() == position);

    math::Quaternion orientation = {1, 2, 3, 4};
    body.SetOrientation(orientation);
    REQUIRE(body.GetOrientation() == orientation);

    math::Vector3 velocity = {1, 2, 3};
    body.SetVelocity(velocity);
    REQUIRE(body.GetVelocity() == velocity);

    math::Vector3 angular_velocity = {1, 2, 3};
    body.SetAngularVelocity(angular_velocity);
    REQUIRE(body.GetAngularVelocity() == angular_velocity);

    math::Vector3 acceleration = {1, 2, 3};
    body.SetAcceleration(acceleration);
    REQUIRE(body.GetAcceleration() == acceleration);
}

TEST_CASE("Body's derived data", "[body]")
{
    Physics::RigidBody body;
    const math::Point3 position = {1, 2, 3};
    const math::Quaternion orientation = math::Quaternion::FromAxisAngleDegrees({1, 0, 0}, 90);
    const math::Transform inverse_inertia_tensor = math::Scale(5);
    body.SetPosition(position);
    body.SetOrientation(orientation);
    body.SetInverseInertiaTensor(inverse_inertia_tensor);
    REQUIRE(body.GetTransform() == math::Transform{});
    REQUIRE(body.GetInverseInertiaTensorWorld() == math::Transform{});
    body.CalculateDerivedData();
    REQUIRE(body.GetTransform() == math::Translate(position) * math::Rotate(orientation));
    REQUIRE(body.GetInverseInertiaTensorWorld() ==
            math::Rotate(orientation) * inverse_inertia_tensor);
}

TEST_CASE("Adding force to the body", "[body]")
{
    SECTION("Applying it to the center of the mass")
    {
        Physics::RigidBody body;
        const math::Vector3 force = {1, 2, 3};
        body.AddForce(force);
        REQUIRE(body.GetAccumulatedForce() == force);
        REQUIRE(body.GetAccumulatedTorque() == math::Vector3::Zero);
        body.AddForce(force);
        REQUIRE(body.GetAccumulatedForce() == force * 2);
        REQUIRE(body.GetAccumulatedTorque() == math::Vector3::Zero);
        body.ClearAccumulators();
        REQUIRE(body.GetAccumulatedForce() == math::Vector3::Zero);
        REQUIRE(body.GetAccumulatedTorque() == math::Vector3::Zero);
    }
    SECTION("Applying it at the point on the body in the world space")
    {
        Physics::RigidBody body;
        body.SetPosition({1, 2, 3});
        const math::Vector3 force = {1, 1, 1};
        body.AddForceAtPoint(force, {0, 0, 0});
        REQUIRE(body.GetAccumulatedForce() == force);
        REQUIRE(body.GetAccumulatedTorque() == math::Vector3{1, -2, 1});
    }
    SECTION("Applying it at the point on the body in the body's local space")
    {
        Physics::RigidBody body;
        body.SetPosition({1, 2, 3});
        body.CalculateDerivedData();
        const math::Vector3 force = {1, 1, 1};
        body.AddForceAtLocalPoint(force, {-1, -2, -3});
        REQUIRE(body.GetAccumulatedForce() == force);
        REQUIRE(body.GetAccumulatedTorque() == math::Vector3{1, -2, 1});
    }
}

TEST_CASE("Applying gravity force generator to the body", "[body][forcegenerator]")
{
    Physics::RigidBody body;
    body.SetMass(5);
    Physics::Gravity gravity{math::Vector3{0, -PHYSICS_REALC(10.0), 0}};
    gravity.UpdateForce(body, 1);
    REQUIRE(body.GetAccumulatedForce() == math::Vector3{0, -PHYSICS_REALC(50.0), 0});
    REQUIRE(body.GetAccumulatedTorque() == math::Vector3::Zero);

    gravity.SetGravity({0, PHYSICS_REALC(5.0), 0});
    REQUIRE(gravity.GetGravity() == math::Vector3{0, PHYSICS_REALC(5.0), 0});
}

TEST_CASE("Applying spring force generator to the body", "[body][forcegenerator]")
{
    SECTION("Creation and getters and setters")
    {
        Physics::RigidBody other_body;
        const math::Point3 connection_point = {1, 2, 3};
        const math::Point3 other_connection_point = {4, 5, 6};
        Physics::Spring S(connection_point, &other_body, other_connection_point, 5, 10);
        REQUIRE(S.GetConnectionPointLocal() == connection_point);
        REQUIRE(S.GetOtherConnectionPointLocal() == other_connection_point);
        REQUIRE(S.GetOtherBody() == &other_body);
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
    SECTION("Spring applying force to the bodies")
    {
        Physics::RigidBody body;
        body.SetPosition({1, 2, 3});
        body.CalculateDerivedData();
        Physics::RigidBody other_body;
        other_body.SetPosition({1, 5, 7});
        other_body.CalculateDerivedData();
        const math::Point3 connection_point = {0, 0, 0};
        Physics::Spring S(connection_point, &other_body, connection_point, 5, 10);
        S.UpdateForce(body, 1);
        REQUIRE(body.GetAccumulatedForce().X == 0);
        REQUIRE(body.GetAccumulatedForce().Z == -20);

#if PHYSICS_REAL_AS_DOUBLE
        constexpr Physics::real k_epsilon = PHYSICS_REALC(1e-6);
#else
        constexpr Physics::real k_epsilon = PHYSICS_REALC(1e-3);
#endif
        REQUIRE(math::IsEqual(body.GetAccumulatedForce().Y, -15, k_epsilon));
    }
}

TEST_CASE("Force registry", "[body][forcegenerator]")
{
    Physics::RigidBody body;
    body.SetMass(5);
    Physics::ForceRegistry registry;
    Physics::Gravity gravity{math::Vector3{0, -PHYSICS_REALC(10.0), 0}};
    registry.Add(&body, &gravity);
    registry.UpdateForces(1);
    REQUIRE(body.GetAccumulatedForce() == math::Vector3{0, -PHYSICS_REALC(50.0), 0});
    REQUIRE(body.GetAccumulatedTorque() == math::Vector3::Zero);
    registry.Remove(&body, &gravity);
    registry.UpdateForces(1);
    REQUIRE(body.GetAccumulatedForce() == math::Vector3{0, -PHYSICS_REALC(50.0), 0});
    REQUIRE(body.GetAccumulatedTorque() == math::Vector3::Zero);
    registry.Add(&body, &gravity);
    registry.Clear();
    registry.UpdateForces(1);
    REQUIRE(body.GetAccumulatedForce() == math::Vector3{0, -PHYSICS_REALC(50.0), 0});
    REQUIRE(body.GetAccumulatedTorque() == math::Vector3::Zero);
}

TEST_CASE("Moving body based on the forces and velocity", "[body][integration]")
{
    SECTION("Force at center of the mass")
    {
        Physics::RigidBody body;
        body.SetMass(2);
        body.SetDamping(0.5);
        body.SetPosition({1, 2, 3});
        body.SetVelocity({1, 2, 3});
        body.AddForce({1, 2, 3});
        body.Integrate(1);
        REQUIRE(body.GetVelocity() ==
                math::Vector3{PHYSICS_REALC(0.75), PHYSICS_REALC(1.5), PHYSICS_REALC(2.25)});
        REQUIRE(body.GetPosition() ==
                math::Point3{PHYSICS_REALC(1.75), PHYSICS_REALC(3.5), PHYSICS_REALC(5.25)});
        REQUIRE(body.GetAccumulatedForce() == math::Vector3::Zero);
        REQUIRE(body.GetTransform() == math::Translate(body.GetPosition()));
    }
    SECTION("Force at the point on the body in world space")
    {
        Physics::RigidBody body;
        body.SetMass(2);
        body.SetDamping(0.5);
        body.SetPosition({1, 2, 3});
        body.SetVelocity({1, 2, 3});
        body.AddForceAtPoint({1, 2, 3}, {1, 1, 1});
        body.Integrate(1);
        REQUIRE(body.GetVelocity() ==
                math::Vector3{PHYSICS_REALC(0.75), PHYSICS_REALC(1.5), PHYSICS_REALC(2.25)});
        REQUIRE(body.GetPosition() ==
                math::Point3{PHYSICS_REALC(1.75), PHYSICS_REALC(3.5), PHYSICS_REALC(5.25)});
        REQUIRE(body.GetAccumulatedForce() == math::Vector3::Zero);
    }
    SECTION("Force at the point on the body in body's local space")
    {
        Physics::RigidBody body;
        body.SetMass(2);
        body.SetDamping(0.5);
        body.SetPosition({1, 2, 3});
        body.SetVelocity({1, 2, 3});
        body.AddForceAtLocalPoint({1, 2, 3}, {1, 1, 1});
        body.Integrate(1);
        REQUIRE(body.GetVelocity() ==
                math::Vector3{PHYSICS_REALC(0.75), PHYSICS_REALC(1.5), PHYSICS_REALC(2.25)});
        REQUIRE(body.GetPosition() ==
                math::Point3{PHYSICS_REALC(1.75), PHYSICS_REALC(3.5), PHYSICS_REALC(5.25)});
        REQUIRE(body.GetAccumulatedForce() == math::Vector3::Zero);
    }
    SECTION("Force that triggers rotational movement as well")
    {
        Physics::RigidBody body;
        body.SetInverseInertiaTensor(math::Scale(3));
        body.CalculateDerivedData();
        body.SetAngularDamping(0.5);
        body.SetOrientation(math::Quaternion::FromAxisAngleDegrees({0, 0, 1}, 90));
        body.AddForceAtPoint({0, 1, 0}, {1, 0, 0});
        REQUIRE(body.GetAccumulatedTorque() == math::Vector3{0, 0, 1});
        body.Integrate(1);
        REQUIRE(body.GetAngularVelocity() == math::Vector3{0, 0, PHYSICS_REALC(1.5)});
        REQUIRE(body.GetAccumulatedTorque() == math::Vector3::Zero);
        const math::Quaternion orientation = body.GetOrientation();
        const math::Vector3 axis = math::Normalize(orientation.Vec);
        Physics::real angle = std::acos(orientation.W) * 2;
        angle = math::Degrees(angle);
        REQUIRE(axis == math::Vector3{0, 0, 1});
        REQUIRE(angle > 90);
    }
}
