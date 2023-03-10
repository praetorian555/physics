#include <catch2/catch_test_macros.hpp>

#include "physics/shapes.h"

TEST_CASE("ShapesSphereCreation")
{
    physics::Sphere S({1, 1, 1}, 5);
    REQUIRE(S.Type == physics::ShapeType::Sphere);
    REQUIRE(S.Center == math::Vector3(1, 1, 1));
    REQUIRE(S.Radius == 5);
    REQUIRE(math::IsEqual(S.GetVolume(), math::kPi * 5 * 5 * 5 * 4 / PHYSICS_REALC(3.0), 0.0001f));
    REQUIRE(S.GetSurfaceArea() == math::kPi * 5 * 5 * 4);
}

TEST_CASE("ShapesSphereIntersection")
{
    {
        const physics::Sphere S1({0, 0, 0}, 1);
        const physics::Sphere S2({0, 0, 0}, 1);
        REQUIRE(S1.Overlaps(S2));
    }
    {
        const physics::Sphere S1({0, 0, 0}, 1);
        const physics::Sphere S2({2, 0, 0}, 2);
        REQUIRE(S1.Overlaps(S2));
    }
    {
        const physics::Sphere S1({0, 0, 0}, 1);
        const physics::Sphere S2({2, 0, 0}, 1);
        REQUIRE(S1.Overlaps(S2));
    }
    {
        const physics::Sphere S1({0, 0, 0}, 2);
        const physics::Sphere S2({1, 0, 0}, 0.5);
        REQUIRE(S1.Overlaps(S2));
    }
    {
        const physics::Sphere S1({0, 0, 0}, 2);
        const physics::Sphere S2({1.5, 0, 0}, 1);
        REQUIRE(S1.Overlaps(S2));
    }
    {
        const physics::Sphere S1({0, 0, 0}, 2);
        const physics::Sphere S2({3, 0, 0}, 0.5);
        REQUIRE(!S1.Overlaps(S2));
    }
}

TEST_CASE("ShapesAABoxConstruction")
{
    {
        physics::AABox B({0, 0, 0}, {1, 1, 1});
        REQUIRE(B.Type == physics::ShapeType::AABox);
        REQUIRE(B.Min == math::Vector3(0, 0, 0));
        REQUIRE(B.Max == math::Vector3(1, 1, 1));
        REQUIRE(B.GetVolume() == 1);
        REQUIRE(B.GetSurfaceArea() == 6);
    }
    {
        const physics::AABox B;
        REQUIRE(B.Type == physics::ShapeType::AABox);
        REQUIRE(B.Min == math::Vector3(0, 0, 0));
        REQUIRE(B.Max == math::Vector3(0, 0, 0));
        REQUIRE(B.GetVolume() == 0);
        REQUIRE(B.GetSurfaceArea() == 0);
    }
}

TEST_CASE("ShapesAABoxIntersection")
{
    {
        const physics::AABox B1({0, 0, 0}, {1, 1, 1});
        const physics::AABox B2({0, 0, 0}, {1, 1, 1});
        REQUIRE(B1.Overlaps(B2));
    }
    {
        const physics::AABox B1({0, 0, 0}, {1, 1, 1});
        const physics::AABox B2({0, 0, 0}, {2, 2, 2});
        REQUIRE(B1.Overlaps(B2));
    }
    {
        const physics::AABox B1({0, 0, 0}, {1, 1, 1});
        const physics::AABox B2({1, 1, 1}, {2, 2, 2});
        REQUIRE(B1.Overlaps(B2));
    }
    {
        const physics::AABox B1({0, 0, 0}, {1, 1, 1});
        const physics::AABox B2({2, 2, 2}, {3, 3, 3});
        REQUIRE(!B1.Overlaps(B2));
    }
}
