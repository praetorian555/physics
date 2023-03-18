#include <catch2/catch_test_macros.hpp>

#include "math/transform.h"

#include "physics/shapes.h"

TEST_CASE("ShapesSphereCreation")
{
    {
        physics::Sphere S({1, 1, 1}, 5);
        REQUIRE(S.Type == physics::ShapeType::Sphere);
        REQUIRE(S.Center == math::Vector3(1, 1, 1));
        REQUIRE(S.Radius == 5);
        REQUIRE(
            math::IsEqual(S.GetVolume(), math::kPi * 5 * 5 * 5 * 4 / PHYSICS_REALC(3.0), 0.0001f));
        REQUIRE(S.GetSurfaceArea() == math::kPi * 5 * 5 * 4);
        REQUIRE(S.IsValid());
    }
    {
        physics::Sphere S{};
        REQUIRE(S.Type == physics::ShapeType::Sphere);
        REQUIRE(S.Center == math::Vector3(0, 0, 0));
        REQUIRE(S.Radius == 0);
        REQUIRE(S.GetVolume() == 0);
        REQUIRE(S.GetSurfaceArea() == 0);
        REQUIRE(!S.IsValid());
    }
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
    {
        const physics::Sphere S1({0, 0, 0}, 2);
        const physics::Sphere S2({0, 0, 0}, 0);
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
        REQUIRE(B.IsValid());
    }
    {
        const physics::AABox B;
        REQUIRE(B.Type == physics::ShapeType::AABox);
        REQUIRE(B.Min == math::Vector3(0, 0, 0));
        REQUIRE(B.Max == math::Vector3(0, 0, 0));
        REQUIRE(B.GetVolume() == 0);
        REQUIRE(B.GetSurfaceArea() == 0);
        REQUIRE(!B.IsValid());
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
    {
        const physics::AABox B1({0, 0, 0}, {2, 2, 2});
        const physics::AABox B2({1, 1, 1}, {1, 1, 1});
        REQUIRE(!B1.Overlaps(B2));
    }
}

TEST_CASE("ShapesPlaneConstruction")
{
    {
        physics::Plane P({0, 0, 3}, 5);
        REQUIRE(P.Type == physics::ShapeType::Plane);
        REQUIRE(P.Normal == math::Vector3(0, 0, 1));
        REQUIRE(P.Distance == 5);
        REQUIRE(P.IsValid());
    }
    {
        const physics::Plane P;
        REQUIRE(P.Type == physics::ShapeType::Plane);
        REQUIRE(P.Normal == math::Vector3(0, 0, 0));
        REQUIRE(P.Distance == 0);
        REQUIRE(!P.IsValid());
    }
    {
        const physics::Plane P = physics::Plane::FromPoints(
            math::Vector3(0, 0, 0), math::Vector3(1, 0, 0), math::Vector3(0, 1, 0));
        REQUIRE(P.Type == physics::ShapeType::Plane);
        REQUIRE(P.Normal == math::Vector3(0, 0, 1));
        REQUIRE(P.Distance == 0);
        REQUIRE(P.IsValid());
    }
}

TEST_CASE("ShapesPlaneIntersection")
{
    {
        const physics::Plane P1({0, 0, 1}, 0);
        const physics::Plane P2({0, 0, 1}, 3);
        REQUIRE(!P1.Overlaps(P2));
    }
    {
        const physics::Plane P1({0, 0, 1}, 0);
        const physics::Plane P2({0, 0, 1}, 0);
        REQUIRE(P1.Overlaps(P2));
    }
    {
        const physics::Plane P1({0, 0, 1}, 0);
        const physics::Plane P2({0, 1, 0}, -3);
        REQUIRE(P1.Overlaps(P2));
    }
    {
        const physics::Plane P1({0, 0, 1}, 0);
        const physics::Plane P2({0, 0, -1}, 3);
        REQUIRE(!P1.Overlaps(P2));
    }
    {
        const physics::Plane P1({0, 0, 1}, 0);
        const physics::Plane P2({0, 0, 0}, 0);
        REQUIRE(!P1.Overlaps(P2));
    }
}

TEST_CASE("ShapesBoxCreation")
{
    {
        physics::Box B;
        REQUIRE(B.Type == physics::ShapeType::Box);
        REQUIRE(B.Center == math::Vector3(0, 0, 0));
        REQUIRE(B.Extents == math::Vector3(0, 0, 0));
        REQUIRE(B.AxisX == math::Vector3(0, 0, 0));
        REQUIRE(B.AxisY == math::Vector3(0, 0, 0));
        REQUIRE(B.AxisZ == math::Vector3(0, 0, 0));
        REQUIRE(!B.IsValid());
    }
    {
        physics::Box B({0, 0, 0}, {1, 1, 1}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1});
        REQUIRE(B.Type == physics::ShapeType::Box);
        REQUIRE(B.Center == math::Vector3(0, 0, 0));
        REQUIRE(B.Extents == math::Vector3(1, 1, 1));
        REQUIRE(B.AxisX == math::Vector3(1, 0, 0));
        REQUIRE(B.AxisY == math::Vector3(0, 1, 0));
        REQUIRE(B.AxisZ == math::Vector3(0, 0, 1));
        REQUIRE(B.IsValid());
        REQUIRE(B.GetSurfaceArea() == 6);
        REQUIRE(B.GetVolume() == 1);
    }
    {
        math::Transform T = math::RotateZ(90);
        physics::Box B({0, 0, 0}, {1, 1, 1}, T.GetMatrix());
        REQUIRE(B.Type == physics::ShapeType::Box);
        REQUIRE(B.Center == math::Vector3(0, 0, 0));
        REQUIRE(B.Extents == math::Vector3(1, 1, 1));
        REQUIRE(math::IsEqual(B.AxisX, math::Vector3(0, 1, 0), PHYSICS_REALC(0.0001)));
        REQUIRE(math::IsEqual(B.AxisY, math::Vector3(-1, 0, 0), PHYSICS_REALC(0.0001)));
        REQUIRE(math::IsEqual(B.AxisZ, math::Vector3(0, 0, 1), PHYSICS_REALC(0.0001)));
        REQUIRE(B.IsValid());
        REQUIRE(B.GetSurfaceArea() == 6);
        REQUIRE(B.GetVolume() == 1);
    }
}

TEST_CASE("ShapesBoxIntersection")
{
    using math::Vector3;

    const physics::Box Box1(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                            Vector3(0, 0, 1));
    const physics::Box Box2(Vector3(3, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                            Vector3(0, 0, 1));
    const physics::Box Box3(Vector3(0.5, 0.5, 0.5), Vector3(1, 1, 1), Vector3(1, 0, 0),
                            Vector3(0, 1, 0), Vector3(0, 0, 1));
    const physics::Box Box4(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(0, 1, 0), Vector3(-1, 0, 0),
                            Vector3(0, 0, 1));
    const physics::Box Box5(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(0, 0, 1), Vector3(0, 1, 0),
                            Vector3(-1, 0, 0));
    const physics::Box Box6(Vector3(2, 2, 2), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                            Vector3(0, 0, 1));
    const physics::Box Box7(Vector3(2, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                            Vector3(0, 0, 1));
    const physics::Box Box8(Vector3(2, 2, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                            Vector3(0, 0, 1));

    SECTION("Non-overlapping boxes with same orientation")
    {
        REQUIRE(!Overlaps(Box1, Box2));
    }
    SECTION("Overlapping boxes with same orientation")
    {
        REQUIRE(Overlaps(Box1, Box3));
    }
    SECTION("Overlapping boxes with different orientations")
    {
        REQUIRE(Overlaps(Box1, Box4));
    }
    SECTION("Non-overlapping boxes with different orientations")
    {
        REQUIRE(!Overlaps(Box2, Box4));
    }
    SECTION("Overlapping boxes with orthogonal orientations")
    {
        REQUIRE(Overlaps(Box1, Box5));
    }
    SECTION("Identical boxes")
    {
        REQUIRE(Overlaps(Box1, Box1));
    }
    SECTION("Boxes touching at one point")
    {
        REQUIRE(Overlaps(Box1, Box6));
    }

    SECTION("Boxes touching along an edge")
    {
        REQUIRE(Overlaps(Box1, Box8));
    }

    SECTION("Boxes touching along a face")
    {
        REQUIRE(Overlaps(Box1, Box7));
    }
}

TEST_CASE("ShapesSpherePlane")
{
    {
        const physics::Sphere S({0, 0, 0}, 1);
        const physics::Plane P({0, 0, 1}, 0);
        REQUIRE(S.Overlaps(P));
    }
    {
        const physics::Sphere S({0, 0, 0}, 1);
        const physics::Plane P({0, 0, 1}, 2);
        REQUIRE(!S.Overlaps(P));
    }
    {
        const physics::Sphere S({0, 0, 0}, 1);
        const physics::Plane P({0, 0, 1}, -2);
        REQUIRE(!S.Overlaps(P));
    }
    {
        const physics::Sphere S({0, 0, 0}, 1);
        const physics::Plane P({0, 0, 1}, 1);
        REQUIRE(S.Overlaps(P));
    }
    {
        const physics::Sphere S({0, 0, 0}, 1);
        const physics::Plane P({0, 0, 1}, -1);
        REQUIRE(S.Overlaps(P));
    }
    {
        const physics::Sphere S({0, 0, 0}, 1);
        const physics::Plane P({0, 0, 0}, 0);
        REQUIRE(!S.Overlaps(P));
    }
    {
        const physics::Sphere S({0, 0, 0}, 0);
        const physics::Plane P({0, 0, 1}, 0);
        REQUIRE(!S.Overlaps(P));
    }
}

TEST_CASE("ShapesAABoxPlane")
{
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({1, 0, 0}, 0);
        REQUIRE(B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({1, 0, 0}, 2);
        REQUIRE(B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({1, 0, 0}, 1);
        REQUIRE(B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({1, 0, 0}, -1);
        REQUIRE(!B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({1, 0, 0}, 5);
        REQUIRE(!B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({0, 0, 0}, 0);
        REQUIRE(!B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {0, 0, 0});
        const physics::Plane P({1, 0, 0}, 0);
        REQUIRE(!B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({0, 1, 0}, 0);
        REQUIRE(B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({0, 1, 0}, 2);
        REQUIRE(B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({0, 1, 0}, 1);
        REQUIRE(B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({0, 1, 0}, -1);
        REQUIRE(!B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({0, 1, 0}, 5);
        REQUIRE(!B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({0, 0, -1}, 0);
        REQUIRE(B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({0, 0, -1}, -2);
        REQUIRE(B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({0, 0, -1}, -1);
        REQUIRE(B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({0, 0, -1}, 1);
        REQUIRE(!B.Overlaps(P));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({0, 0, -1}, -5);
        REQUIRE(!B.Overlaps(P));
    }
}

TEST_CASE("OverlapsAABoxSphere")
{
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Sphere S({0, 0, 0}, 1);
        REQUIRE(B.Overlaps(S));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Sphere S({-1, -1, -1}, 2);
        REQUIRE(B.Overlaps(S));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Sphere S({-1, -1, -1}, 1);
        REQUIRE(!B.Overlaps(S));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Sphere S({-2, 0, 0}, 1);
        REQUIRE(!B.Overlaps(S));
    }
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Sphere S({1, 1, 1}, 1);
        REQUIRE(B.Overlaps(S));
    }
}

TEST_CASE("DistancePointPlane")
{
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Plane P2({0, 0, 1}, 0);
        REQUIRE(physics::Distance(P, P2) == 0);
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Plane P2({0, 0, 1}, 1);
        REQUIRE(physics::Distance(P, P2) == -1);
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Plane P2({0, 0, 1}, -1);
        REQUIRE(physics::Distance(P, P2) == 1);
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Plane P2({0, 0, -1}, 1);
        REQUIRE(physics::Distance(P, P2) == -1);
    }
}

TEST_CASE("DistancePointAABox")
{
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        REQUIRE(physics::Distance(P, B) == 0);
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({1, 1, 1}, {2, 2, 2});
        REQUIRE(physics::Distance(P, B) == math::Sqrt(3));
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({-1, -1, -1}, {2, 2, 2});
        REQUIRE(physics::Distance(P, B) == 0);
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({-2, -2, -2}, {-1, -1, -1});
        REQUIRE(physics::Distance(P, B) == math::Sqrt(3));
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({1, -2, -2}, {2, 2, 2});
        REQUIRE(physics::Distance(P, B) == 1);
    }
}

TEST_CASE("DistancePointSphere")
{
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Sphere S({0, 0, 0}, 1);
        REQUIRE(physics::Distance(P, S) == 0);
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Sphere S({1, 1, 1}, 1);
        REQUIRE(physics::Distance(P, S) == (math::Sqrt(3) - 1));
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Sphere S({-1, -1, -1}, 1);
        REQUIRE(physics::Distance(P, S) == (math::Sqrt(3) - 1));
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Sphere S({1, 1, 1}, 2);
        REQUIRE(physics::Distance(P, S) == 0);
    }
}

TEST_CASE("ClosestPointPointPlane")
{
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Plane P2({0, 0, 1}, 0);
        REQUIRE(physics::ClosestPoint(P, P2) == math::Vector3(0, 0, 0));
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Plane P2({0, 0, 1}, 1);
        REQUIRE(physics::ClosestPoint(P, P2) == math::Vector3(0, 0, 1));
    }
    {
        const math::Vector3 P(0, 2, 0);
        const physics::Plane P2({0, 0, 1}, -1);
        REQUIRE(physics::ClosestPoint(P, P2) == math::Vector3(0, 2, -1));
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Plane P2({0, 0, -1}, 1);
        REQUIRE(physics::ClosestPoint(P, P2) == math::Vector3(0, 0, -1));
    }
}

TEST_CASE("ClosestPointPointAABox")
{
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        REQUIRE(physics::ClosestPoint(P, B) == math::Vector3(0, 0, 0));
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({1, 1, 1}, {2, 2, 2});
        REQUIRE(physics::ClosestPoint(P, B) == math::Vector3(1, 1, 1));
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({-1, -1, -1}, {2, 2, 2});
        REQUIRE(physics::ClosestPoint(P, B) == math::Vector3(0, 0, 0));
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({-2, -2, -2}, {-1, -1, -1});
        REQUIRE(physics::ClosestPoint(P, B) == math::Vector3(-1, -1, -1));
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({1, -5, -5}, {5, 5, 5});
        REQUIRE(physics::ClosestPoint(P, B) == math::Vector3(1, 0, 0));
    }
}

TEST_CASE("ClosestPointPointSphere")
{
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Sphere S({0, 0, 0}, 1);
        REQUIRE(physics::ClosestPoint(P, S) == math::Vector3(0, 0, 0));
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Sphere S({1, 0, 0}, 1);
        REQUIRE(physics::ClosestPoint(P, S) == math::Vector3(0, 0, 0));
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Sphere S({-2, 0, 0}, 1);
        REQUIRE(physics::ClosestPoint(P, S) == math::Vector3(-1, 0, 0));
    }
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Sphere S({1, 1, 1}, 2);
        REQUIRE(physics::ClosestPoint(P, S) == math::Vector3(0, 0, 0));
    }
}
