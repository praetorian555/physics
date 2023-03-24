#include <catch2/catch_test_macros.hpp>

#include "math/transform.h"

#include "physics/shapes.h"

TEST_CASE("Creation of a sphere shape", "[creation][shape][sphere]")
{
    SECTION("Default constructor")
    {
        Physics::Sphere s;
        REQUIRE(s.type == Physics::ShapeType::Sphere);
        REQUIRE(s.center == math::Point3(0, 0, 0));
        REQUIRE(s.radius == 0);
        REQUIRE(s.GetVolume() == 0);
        REQUIRE(s.GetSurfaceArea() == 0);
        REQUIRE(s.IsValid());
    }
    SECTION("Constructor with center and radius")
    {
        Physics::Sphere s({1, 1, 1}, 5);
        REQUIRE(s.type == Physics::ShapeType::Sphere);
        REQUIRE(s.center == math::Point3(1, 1, 1));
        REQUIRE(s.radius == 5);
        REQUIRE(
            math::IsEqual(s.GetVolume(), math::kPi * 5 * 5 * 5 * 4 / PHYSICS_REALC(3.0), 0.0001f));
        REQUIRE(s.GetSurfaceArea() == math::kPi * 5 * 5 * 4);
        REQUIRE(s.IsValid());
    }
    SECTION("Constructor with radius 0")
    {
        Physics::Sphere s({1, 1, 1}, 0);
        REQUIRE(s.type == Physics::ShapeType::Sphere);
        REQUIRE(s.center == math::Point3(1, 1, 1));
        REQUIRE(s.radius == 0);
        REQUIRE(s.GetVolume() == 0);
        REQUIRE(s.GetSurfaceArea() == 0);
        REQUIRE(s.IsValid());
    }
    SECTION("Constructor with center and invalid radius")
    {
        Physics::Sphere s({1, 1, 1}, -5);
        REQUIRE(s.type == Physics::ShapeType::Sphere);
        REQUIRE(s.center == math::Point3(1, 1, 1));
        REQUIRE(s.radius == -5);
        REQUIRE(!s.IsValid());
    }
}

TEST_CASE("Creation of a axis-aligned box shape", "[creation][shape][aabox]")
{
    SECTION("Default constructor")
    {
        const Physics::AABox b;
        REQUIRE(b.type == Physics::ShapeType::AABox);
        REQUIRE(b.min == math::Point3(0, 0, 0));
        REQUIRE(b.max == math::Point3(0, 0, 0));
        REQUIRE(b.GetVolume() == 0);
        REQUIRE(b.GetSurfaceArea() == 0);
        REQUIRE(b.IsValid());
    }
    SECTION("Constructor with valid min and max")
    {
        Physics::AABox b({0, 0, 0}, {1, 1, 1});
        REQUIRE(b.type == Physics::ShapeType::AABox);
        REQUIRE(b.min == math::Point3(0, 0, 0));
        REQUIRE(b.max == math::Point3(1, 1, 1));
        REQUIRE(b.GetVolume() == 1);
        REQUIRE(b.GetSurfaceArea() == 6);
        REQUIRE(b.IsValid());
    }
    SECTION("Constructor with equal min and max")
    {
        Physics::AABox b({0, 0, 0}, {0, 0, 0});
        REQUIRE(b.type == Physics::ShapeType::AABox);
        REQUIRE(b.min == math::Point3(0, 0, 0));
        REQUIRE(b.max == math::Point3(0, 0, 0));
        REQUIRE(b.GetVolume() == 0);
        REQUIRE(b.GetSurfaceArea() == 0);
        REQUIRE(b.IsValid());
    }
    SECTION("Constructor with invalid min and max")
    {
        Physics::AABox b({1, 1, 1}, {0, 0, 0});
        REQUIRE(b.type == Physics::ShapeType::AABox);
        REQUIRE(b.min == math::Point3(1, 1, 1));
        REQUIRE(b.max == math::Point3(0, 0, 0));
        REQUIRE(!b.IsValid());
    }
}

TEST_CASE("Creation of a plane shape", "[creation][shape][plane]")
{
    SECTION("Default constructor")
    {
        const Physics::Plane p;
        REQUIRE(p.type == Physics::ShapeType::Plane);
        REQUIRE(p.normal == math::Vector3(0, 0, 0));
        REQUIRE(p.distance == 0);
        REQUIRE(!p.IsValid());
    }
    SECTION("Constructor with normalized normal")
    {
        Physics::Plane p({0, 0, 1}, 5);
        REQUIRE(p.type == Physics::ShapeType::Plane);
        REQUIRE(p.normal == math::Vector3(0, 0, 1));
        REQUIRE(p.distance == 5);
        REQUIRE(p.IsValid());
    }
    SECTION("Constructor with non-normalized normal")
    {
        Physics::Plane p({0, 0, 3}, 5);
        REQUIRE(p.type == Physics::ShapeType::Plane);
        REQUIRE(p.normal == math::Vector3(0, 0, 1));
        REQUIRE(p.distance == 5);
        REQUIRE(p.IsValid());
    }
    SECTION("Constructor with invalid normal")
    {
        Physics::Plane p({0, 0, 0}, 5);
        REQUIRE(p.type == Physics::ShapeType::Plane);
        REQUIRE(p.normal == math::Vector3(0, 0, 0));
        REQUIRE(p.distance == 5);
        REQUIRE(!p.IsValid());
    }
    SECTION("Construction of a plane from three points that are not collinear")
    {
        const Physics::Plane p = Physics::Plane::FromPoints(
            math::Point3(0, 0, 0), math::Point3(1, 0, 0), math::Point3(0, 1, 0));
        REQUIRE(p.type == Physics::ShapeType::Plane);
        REQUIRE(p.normal == math::Vector3(0, 0, 1));
        REQUIRE(p.distance == 0);
        REQUIRE(p.IsValid());
    }
    SECTION("Construction of a plane from three points that are collinear")
    {
        const Physics::Plane p = Physics::Plane::FromPoints(
            math::Point3(0, 0, 0), math::Point3(1, 0, 0), math::Point3(2, 0, 0));
        REQUIRE(p.type == Physics::ShapeType::Plane);
        REQUIRE(p.normal == math::Vector3(0, 0, 0));
        REQUIRE(p.distance == 0);
        REQUIRE(!p.IsValid());
    }
}

TEST_CASE("Creation of a oriented box shape", "[creation][shape][box]")
{
    SECTION("Default constructor")
    {
        Physics::Box b;
        REQUIRE(b.type == Physics::ShapeType::Box);
        REQUIRE(b.center == math::Point3(0, 0, 0));
        REQUIRE(b.half_extents == math::Vector3(0, 0, 0));
        REQUIRE(b.axis_x == math::Vector3(0, 0, 0));
        REQUIRE(b.axis_y == math::Vector3(0, 0, 0));
        REQUIRE(b.axis_z == math::Vector3(0, 0, 0));
        REQUIRE(!b.IsValid());
    }
    SECTION("Constructor with valid parameters and axes vectors")
    {
        Physics::Box b({0, 0, 0}, {1, 1, 1}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1});
        REQUIRE(b.type == Physics::ShapeType::Box);
        REQUIRE(b.center == math::Point3(0, 0, 0));
        REQUIRE(b.half_extents == math::Vector3(1, 1, 1));
        REQUIRE(b.axis_x == math::Vector3(1, 0, 0));
        REQUIRE(b.axis_y == math::Vector3(0, 1, 0));
        REQUIRE(b.axis_z == math::Vector3(0, 0, 1));
        REQUIRE(b.IsValid());
        REQUIRE(b.GetSurfaceArea() == 6);
        REQUIRE(b.GetVolume() == 1);
    }
    SECTION("Constructor with zero extent")
    {
        Physics::Box b({0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1});
        REQUIRE(b.type == Physics::ShapeType::Box);
        REQUIRE(b.center == math::Point3(0, 0, 0));
        REQUIRE(b.half_extents == math::Vector3(0, 0, 0));
        REQUIRE(b.axis_x == math::Vector3(1, 0, 0));
        REQUIRE(b.axis_y == math::Vector3(0, 1, 0));
        REQUIRE(b.axis_z == math::Vector3(0, 0, 1));
        REQUIRE(b.IsValid());
        REQUIRE(b.GetSurfaceArea() == 0);
        REQUIRE(b.GetVolume() == 0);
    }
    SECTION("Constructor with invalid extent")
    {
        Physics::Box b({0, 0, 0}, {-1, -1, -1}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1});
        REQUIRE(b.type == Physics::ShapeType::Box);
        REQUIRE(b.center == math::Point3(0, 0, 0));
        REQUIRE(b.half_extents == math::Vector3(-1, -1, -1));
        REQUIRE(b.axis_x == math::Vector3(1, 0, 0));
        REQUIRE(b.axis_y == math::Vector3(0, 1, 0));
        REQUIRE(b.axis_z == math::Vector3(0, 0, 1));
        REQUIRE(!b.IsValid());
    }
    SECTION("Constructor with valid parameters and non-normalized axes vectors")
    {
        Physics::Box b({0, 0, 0}, {1, 1, 1}, {3, 0, 0}, {0, 5, 0}, {0, 0, 2});
        REQUIRE(b.type == Physics::ShapeType::Box);
        REQUIRE(b.center == math::Point3(0, 0, 0));
        REQUIRE(b.half_extents == math::Vector3(1, 1, 1));
        REQUIRE(b.axis_x == math::Vector3(1, 0, 0));
        REQUIRE(b.axis_y == math::Vector3(0, 1, 0));
        REQUIRE(b.axis_z == math::Vector3(0, 0, 1));
        REQUIRE(b.IsValid());
        REQUIRE(b.GetSurfaceArea() == 6);
        REQUIRE(b.GetVolume() == 1);
    }
    SECTION("Constructor with valid parameters and rotation matrix")
    {
        const math::Transform t = math::RotateZ(90);
        Physics::Box b({0, 0, 0}, {1, 1, 1}, t.GetMatrix());
        REQUIRE(b.type == Physics::ShapeType::Box);
        REQUIRE(b.center == math::Point3(0, 0, 0));
        REQUIRE(b.half_extents == math::Vector3(1, 1, 1));
        REQUIRE(math::IsEqual(b.axis_x, math::Vector3(0, 1, 0), PHYSICS_REALC(0.0001)));
        REQUIRE(math::IsEqual(b.axis_y, math::Vector3(-1, 0, 0), PHYSICS_REALC(0.0001)));
        REQUIRE(math::IsEqual(b.axis_z, math::Vector3(0, 0, 1), PHYSICS_REALC(0.0001)));
        REQUIRE(b.IsValid());
        REQUIRE(b.GetSurfaceArea() == 6);
        REQUIRE(b.GetVolume() == 1);
    }
    SECTION("Constructor with invalid extent and rotation matrix")
    {
        const math::Transform t = math::RotateZ(90);
        Physics::Box b({0, 0, 0}, {-1, -1, -1}, t.GetMatrix());
        REQUIRE(b.type == Physics::ShapeType::Box);
        REQUIRE(b.center == math::Point3(0, 0, 0));
        REQUIRE(b.half_extents == math::Vector3(-1, -1, -1));
        REQUIRE(math::IsEqual(b.axis_x, math::Vector3(0, 1, 0), PHYSICS_REALC(0.0001)));
        REQUIRE(math::IsEqual(b.axis_y, math::Vector3(-1, 0, 0), PHYSICS_REALC(0.0001)));
        REQUIRE(math::IsEqual(b.axis_z, math::Vector3(0, 0, 1), PHYSICS_REALC(0.0001)));
        REQUIRE(!b.IsValid());
    }
}
