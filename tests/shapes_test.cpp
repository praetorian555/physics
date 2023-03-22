#include <catch2/catch_test_macros.hpp>

#include "math/transform.h"

#include "physics/shapes.h"

TEST_CASE("Creation of a sphere shape", "[creation][shape][sphere]")
{
    SECTION("Default constructor")
    {
        physics::Sphere S;
        REQUIRE(S.Type == physics::ShapeType::Sphere);
        REQUIRE(S.Center == math::Vector3(0, 0, 0));
        REQUIRE(S.Radius == 0);
        REQUIRE(S.GetVolume() == 0);
        REQUIRE(S.GetSurfaceArea() == 0);
        REQUIRE(S.IsValid());
    }
    SECTION("Constructor with center and radius")
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
    SECTION("Constructor with radius 0")
    {
        physics::Sphere S({1, 1, 1}, 0);
        REQUIRE(S.Type == physics::ShapeType::Sphere);
        REQUIRE(S.Center == math::Vector3(1, 1, 1));
        REQUIRE(S.Radius == 0);
        REQUIRE(S.GetVolume() == 0);
        REQUIRE(S.GetSurfaceArea() == 0);
        REQUIRE(S.IsValid());
    }
    SECTION("Constructor with center and invalid radius")
    {
        physics::Sphere S({1, 1, 1}, -5);
        REQUIRE(S.Type == physics::ShapeType::Sphere);
        REQUIRE(S.Center == math::Vector3(1, 1, 1));
        REQUIRE(S.Radius == -5);
        REQUIRE(!S.IsValid());
    }
}

TEST_CASE("Creation of a axis-aligned box shape", "[creation][shape][aabox]")
{
    SECTION("Default constructor")
    {
        const physics::AABox B;
        REQUIRE(B.Type == physics::ShapeType::AABox);
        REQUIRE(B.Min == math::Vector3(0, 0, 0));
        REQUIRE(B.Max == math::Vector3(0, 0, 0));
        REQUIRE(B.GetVolume() == 0);
        REQUIRE(B.GetSurfaceArea() == 0);
        REQUIRE(B.IsValid());
    }
    SECTION("Constructor with valid min and max")
    {
        physics::AABox B({0, 0, 0}, {1, 1, 1});
        REQUIRE(B.Type == physics::ShapeType::AABox);
        REQUIRE(B.Min == math::Vector3(0, 0, 0));
        REQUIRE(B.Max == math::Vector3(1, 1, 1));
        REQUIRE(B.GetVolume() == 1);
        REQUIRE(B.GetSurfaceArea() == 6);
        REQUIRE(B.IsValid());
    }
    SECTION("Constructor with equal min and max")
    {
        physics::AABox B({0, 0, 0}, {0, 0, 0});
        REQUIRE(B.Type == physics::ShapeType::AABox);
        REQUIRE(B.Min == math::Vector3(0, 0, 0));
        REQUIRE(B.Max == math::Vector3(0, 0, 0));
        REQUIRE(B.GetVolume() == 0);
        REQUIRE(B.GetSurfaceArea() == 0);
        REQUIRE(B.IsValid());
    }
    SECTION("Constructor with invalid min and max")
    {
        physics::AABox B({1, 1, 1}, {0, 0, 0});
        REQUIRE(B.Type == physics::ShapeType::AABox);
        REQUIRE(B.Min == math::Vector3(1, 1, 1));
        REQUIRE(B.Max == math::Vector3(0, 0, 0));
        REQUIRE(!B.IsValid());
    }
}

TEST_CASE("Creation of a plane shape", "[creation][shape][plane]")
{
    SECTION("Default constructor")
    {
        const physics::Plane P;
        REQUIRE(P.Type == physics::ShapeType::Plane);
        REQUIRE(P.Normal == math::Vector3(0, 0, 0));
        REQUIRE(P.Distance == 0);
        REQUIRE(!P.IsValid());
    }
    SECTION("Constructor with normalized normal")
    {
        physics::Plane P({0, 0, 1}, 5);
        REQUIRE(P.Type == physics::ShapeType::Plane);
        REQUIRE(P.Normal == math::Vector3(0, 0, 1));
        REQUIRE(P.Distance == 5);
        REQUIRE(P.IsValid());
    }
    SECTION("Constructor with non-normalized normal")
    {
        physics::Plane P({0, 0, 3}, 5);
        REQUIRE(P.Type == physics::ShapeType::Plane);
        REQUIRE(P.Normal == math::Vector3(0, 0, 1));
        REQUIRE(P.Distance == 5);
        REQUIRE(P.IsValid());
    }
    SECTION("Constructor with invalid normal")
    {
        physics::Plane P({0, 0, 0}, 5);
        REQUIRE(P.Type == physics::ShapeType::Plane);
        REQUIRE(P.Normal == math::Vector3(0, 0, 0));
        REQUIRE(P.Distance == 5);
        REQUIRE(!P.IsValid());
    }
    SECTION("Construction of a plane from three points that are not collinear")
    {
        const physics::Plane P = physics::Plane::FromPoints(
            math::Vector3(0, 0, 0), math::Vector3(1, 0, 0), math::Vector3(0, 1, 0));
        REQUIRE(P.Type == physics::ShapeType::Plane);
        REQUIRE(P.Normal == math::Vector3(0, 0, 1));
        REQUIRE(P.Distance == 0);
        REQUIRE(P.IsValid());
    }
    SECTION("Construction of a plane from three points that are collinear")
    {
        const physics::Plane P = physics::Plane::FromPoints(
            math::Vector3(0, 0, 0), math::Vector3(1, 0, 0), math::Vector3(2, 0, 0));
        REQUIRE(P.Type == physics::ShapeType::Plane);
        REQUIRE(P.Normal == math::Vector3(0, 0, 0));
        REQUIRE(P.Distance == 0);
        REQUIRE(!P.IsValid());
    }
}

TEST_CASE("Creation of a oriented box shape", "[creation][shape][box]")
{
    SECTION("Default constructor")
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
    SECTION("Constructor with valid parameters and axes vectors")
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
    SECTION("Constructor with zero extent")
    {
        physics::Box B({0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1});
        REQUIRE(B.Type == physics::ShapeType::Box);
        REQUIRE(B.Center == math::Vector3(0, 0, 0));
        REQUIRE(B.Extents == math::Vector3(0, 0, 0));
        REQUIRE(B.AxisX == math::Vector3(1, 0, 0));
        REQUIRE(B.AxisY == math::Vector3(0, 1, 0));
        REQUIRE(B.AxisZ == math::Vector3(0, 0, 1));
        REQUIRE(B.IsValid());
        REQUIRE(B.GetSurfaceArea() == 0);
        REQUIRE(B.GetVolume() == 0);
    }
    SECTION("Constructor with invalid extent")
    {
        physics::Box B({0, 0, 0}, {-1, -1, -1}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1});
        REQUIRE(B.Type == physics::ShapeType::Box);
        REQUIRE(B.Center == math::Vector3(0, 0, 0));
        REQUIRE(B.Extents == math::Vector3(-1, -1, -1));
        REQUIRE(B.AxisX == math::Vector3(1, 0, 0));
        REQUIRE(B.AxisY == math::Vector3(0, 1, 0));
        REQUIRE(B.AxisZ == math::Vector3(0, 0, 1));
        REQUIRE(!B.IsValid());
    }
    SECTION("Constructor with valid parameters and non-normalized axes vectors")
    {
        physics::Box B({0, 0, 0}, {1, 1, 1}, {3, 0, 0}, {0, 5, 0}, {0, 0, 2});
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
    SECTION("Constructor with valid parameters and rotation matrix")
    {
        const math::Transform T = math::RotateZ(90);
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
    SECTION("Constructor with invalid extent and rotation matrix")
    {
        const math::Transform T = math::RotateZ(90);
        physics::Box B({0, 0, 0}, {-1, -1, -1}, T.GetMatrix());
        REQUIRE(B.Type == physics::ShapeType::Box);
        REQUIRE(B.Center == math::Vector3(0, 0, 0));
        REQUIRE(B.Extents == math::Vector3(-1, -1, -1));
        REQUIRE(math::IsEqual(B.AxisX, math::Vector3(0, 1, 0), PHYSICS_REALC(0.0001)));
        REQUIRE(math::IsEqual(B.AxisY, math::Vector3(-1, 0, 0), PHYSICS_REALC(0.0001)));
        REQUIRE(math::IsEqual(B.AxisZ, math::Vector3(0, 0, 1), PHYSICS_REALC(0.0001)));
        REQUIRE(!B.IsValid());
    }
}

TEST_CASE("Overlap of two spheres", "[overlaps][shape][sphere]")
{
    SECTION("Identical spheres")
    {
        const physics::Sphere S1({0, 0, 0}, 1);
        const physics::Sphere S2({0, 0, 0}, 1);
        REQUIRE(S1.Overlaps(S2));
    }
    SECTION("Overlapping spheres where center is on the surface of the other sphere")
    {
        const physics::Sphere S1({0, 0, 0}, 1);
        const physics::Sphere S2({2, 0, 0}, 2);
        REQUIRE(S1.Overlaps(S2));
    }
    SECTION("Spheres overlapping at one point")
    {
        const physics::Sphere S1({0, 0, 0}, 1);
        const physics::Sphere S2({2, 0, 0}, 1);
        REQUIRE(S1.Overlaps(S2));
    }
    SECTION("Spheres overlapping where one sphere is contained in another")
    {
        const physics::Sphere S1({0, 0, 0}, 2);
        const physics::Sphere S2({1, 0, 0}, 0.5);
        REQUIRE(S1.Overlaps(S2));
    }
    SECTION("Overlapping spheres where centers are not contained in the other sphere")
    {
        const physics::Sphere S1({0, 0, 0}, 2);
        const physics::Sphere S2({1.5, 0, 0}, 1);
        REQUIRE(S1.Overlaps(S2));
    }
    SECTION("Non-overlapping spheres")
    {
        const physics::Sphere S1({0, 0, 0}, 2);
        const physics::Sphere S2({3, 0, 0}, 0.5);
        REQUIRE(!S1.Overlaps(S2));
    }
    SECTION("Overlapping zero radius sphere")
    {
        const physics::Sphere S1({0, 0, 0}, 2);
        const physics::Sphere S2({0, 0, 0}, 0);
        REQUIRE(S1.Overlaps(S2));
    }
    SECTION("Non-overlapping zero radius sphere")
    {
        const physics::Sphere S1({0, 0, 0}, 2);
        const physics::Sphere S2({3, 3, 3}, 0);
        REQUIRE(!S1.Overlaps(S2));
    }
}

TEST_CASE("Overlap of two axis-aligned boxes", "[overlaps][shape][aabox]")
{
    SECTION("Identical boxes")
    {
        const physics::AABox B1({0, 0, 0}, {1, 1, 1});
        const physics::AABox B2({0, 0, 0}, {1, 1, 1});
        REQUIRE(B1.Overlaps(B2));
    }
    SECTION("One box contains the other")
    {
        const physics::AABox B1({0, 0, 0}, {1, 1, 1});
        const physics::AABox B2({0, 0, 0}, {2, 2, 2});
        REQUIRE(B1.Overlaps(B2));
    }
    SECTION("Overlapping at one vertex")
    {
        const physics::AABox B1({0, 0, 0}, {1, 1, 1});
        const physics::AABox B2({1, 1, 1}, {2, 2, 2});
        REQUIRE(B1.Overlaps(B2));
    }
    SECTION("Overlapping at one edge")
    {
        const physics::AABox B1({0, 0, 0}, {1, 1, 1});
        const physics::AABox B2({1, 0, 1}, {2, 2, 2});
        REQUIRE(B1.Overlaps(B2));
    }
    SECTION("Overlapping at one face")
    {
        const physics::AABox B1({0, 0, 0}, {1, 1, 1});
        const physics::AABox B2({1, 0, 0}, {2, 1, 2});
        REQUIRE(B1.Overlaps(B2));
    }
    SECTION("Overlapping but no center is contained in the other box")
    {
        const physics::AABox B1({0, 0, 0}, {1, 1, 1});
        const physics::AABox B2({0.5, 0.5, 0.5}, {2, 2, 2});
        REQUIRE(B1.Overlaps(B2));
    }
    SECTION("Non-overlapping boxes")
    {
        const physics::AABox B1({0, 0, 0}, {1, 1, 1});
        const physics::AABox B2({2, 2, 2}, {3, 3, 3});
        REQUIRE(!B1.Overlaps(B2));
    }
    SECTION("Overlapping a point box")
    {
        const physics::AABox B1({0, 0, 0}, {2, 2, 2});
        const physics::AABox B2({1, 1, 1}, {1, 1, 1});
        REQUIRE(B1.Overlaps(B2));
    }
    SECTION("Non-overlapping point box")
    {
        const physics::AABox B1({0, 0, 0}, {2, 2, 2});
        const physics::AABox B2({3, 3, 3}, {3, 3, 3});
        REQUIRE(!B1.Overlaps(B2));
    }
}

TEST_CASE("Overlap of two planes", "[overlaps][shape][plane]")
{
    SECTION("Parallel planes")
    {
        const physics::Plane P1({0, 0, 1}, 0);
        const physics::Plane P2({0, 0, 1}, 3);
        REQUIRE(!P1.Overlaps(P2));
    }
    SECTION("Identical planes")
    {
        const physics::Plane P1({0, 0, 1}, 0);
        const physics::Plane P2({0, 0, 1}, 0);
        REQUIRE(P1.Overlaps(P2));
    }
    SECTION("Parallel planes with opposite normals")
    {
        const physics::Plane P1({0, 0, 1}, 0);
        const physics::Plane P2({0, 0, -1}, 3);
        REQUIRE(!P1.Overlaps(P2));
    }
    SECTION("Parallel planes with opposite normals but same offsets")
    {
        const physics::Plane P1({0, 0, 1}, 0);
        const physics::Plane P2({0, 0, -1}, 0);
        REQUIRE(P1.Overlaps(P2));
    }
    SECTION("Overlapping planes")
    {
        const physics::Plane P1({0, 0, 1}, 0);
        const physics::Plane P2({0, 1, 0}, -3);
        REQUIRE(P1.Overlaps(P2));
    }
}

TEST_CASE("Overlap of two oriented boxes", "[overlaps][shape][box]")
{
    using math::Vector3;

    SECTION("Non-overlapping Boxes with same orientation")
    {
        const physics::Box Box1(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const physics::Box Box2(Vector3(3, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        REQUIRE(!Overlaps(Box1, Box2));
    }
    SECTION("Overlapping Boxes with same orientation")
    {
        const physics::Box Box1(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const physics::Box Box2(Vector3(0.5, 0.5, 0.5), Vector3(1, 1, 1), Vector3(1, 0, 0),
                                Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(Box1, Box2));
    }
    SECTION("Overlapping Boxes with different orientations")
    {
        const physics::Box Box1(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const physics::Box Box2(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(0, 1, 0), Vector3(-1, 0, 0),
                                Vector3(0, 0, 1));
        REQUIRE(Overlaps(Box1, Box2));
    }
    SECTION("Non-overlapping Boxes with different orientations")
    {
        const physics::Box Box1(Vector3(3, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const physics::Box Box2(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(0, 1, 0), Vector3(-1, 0, 0),
                                Vector3(0, 0, 1));
        REQUIRE(!Overlaps(Box1, Box2));
    }
    SECTION("Overlapping Boxes with orthogonal orientations")
    {
        const physics::Box Box1(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const physics::Box Box2(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(0, 0, 1), Vector3(0, 1, 0),
                                Vector3(-1, 0, 0));
        REQUIRE(Overlaps(Box1, Box2));
    }
    SECTION("Identical Boxes")
    {
        const physics::Box Box1(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        REQUIRE(Overlaps(Box1, Box1));
    }
    SECTION("Boxes touching at one Point")
    {
        const physics::Box Box1(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const physics::Box Box2(Vector3(2, 2, 2), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        REQUIRE(Overlaps(Box1, Box2));
    }
    SECTION("Boxes touching along an edge")
    {
        const physics::Box Box1(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const physics::Box Box2(Vector3(2, 2, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        REQUIRE(Overlaps(Box1, Box2));
    }
    SECTION("Boxes touching along a face")
    {
        const physics::Box Box1(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const physics::Box Box2(Vector3(2, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        REQUIRE(Overlaps(Box1, Box2));
    }
    SECTION("Overlapping point box")
    {
        const physics::Box Box1(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const physics::Box Box2(Vector3(1, 1, 1), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        REQUIRE(Overlaps(Box1, Box2));
    }
    SECTION("Non-overlapping point box")
    {
        const physics::Box Box1(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const physics::Box Box2(Vector3(2, 2, 2), Vector3(2, 2, 2), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        REQUIRE(Overlaps(Box1, Box2));
    }
}

TEST_CASE("Overlap of a sphere and a plane", "[overlaps][shape][sphere][plane]")
{
    SECTION("Overlapping")
    {
        const physics::Sphere S({0, 0, 0}, 1);
        const physics::Plane P({0, 0, 1}, 0);
        REQUIRE(S.Overlaps(P));
    }
    SECTION("Non-overlapping")
    {
        const physics::Sphere S({0, 0, 0}, 1);
        const physics::Plane P({0, 0, 1}, 2);
        REQUIRE(!S.Overlaps(P));
    }
    SECTION("Non-overlapping where plane normal is in negative direction")
    {
        const physics::Sphere S({0, 0, 0}, 1);
        const physics::Plane P({0, 0, 1}, -2);
        REQUIRE(!S.Overlaps(P));
    }
    SECTION("Overlapping at one point")
    {
        const physics::Sphere S({0, 0, 0}, 1);
        const physics::Plane P({0, 0, 1}, 1);
        REQUIRE(S.Overlaps(P));
    }
    SECTION("Overlapping at one point on the other side of sphere")
    {
        const physics::Sphere S({0, 0, 0}, 1);
        const physics::Plane P({0, 0, 1}, -1);
        REQUIRE(S.Overlaps(P));
    }
    SECTION("Overlapping point sphere")
    {
        const physics::Sphere S({0, 0, 0}, 0);
        const physics::Plane P({0, 0, 1}, 0);
        REQUIRE(S.Overlaps(P));
    }
    SECTION("Non-overlapping point sphere")
    {
        const physics::Sphere S({0, 0, 0}, 0);
        const physics::Plane P({0, 0, 1}, 1);
        REQUIRE(!S.Overlaps(P));
    }
}

TEST_CASE("Overlap of an axis-aligned box and a plane", "[overlaps][shape][aabox][plane]")
{
    SECTION("Overlapping on a face")
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({1, 0, 0}, 0);
        REQUIRE(B.Overlaps(P));
    }
    SECTION("Overlapping on a vertex")
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({1, 1, 1}, 0);
        REQUIRE(B.Overlaps(P));
    }
    SECTION("Overlapping on an edge")
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({1, 1, 0}, 0);
        REQUIRE(B.Overlaps(P));
    }
    SECTION("Overlapping on a face from the negative side of a plane")
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({1, 0, 0}, 2);
        REQUIRE(B.Overlaps(P));
    }
    SECTION("Overlapping")
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({1, 1, 1}, 1);
        REQUIRE(B.Overlaps(P));
    }
    SECTION("Non-overlapping on positive side")
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({1, 1, 1}, -1);
        REQUIRE(!B.Overlaps(P));
    }
    SECTION("Non-overlapping on negative side")
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Plane P({1, 1, 1}, 5);
        REQUIRE(!B.Overlaps(P));
    }
    SECTION("Overlapping point box")
    {
        const physics::AABox B({0, 0, 0}, {0, 0, 0});
        const physics::Plane P({1, 0, 0}, 0);
        REQUIRE(B.Overlaps(P));
    }
    SECTION("Non-overlapping point box")
    {
        const physics::AABox B({0, 0, 0}, {0, 0, 0});
        const physics::Plane P({1, 0, 0}, 1);
        REQUIRE(!B.Overlaps(P));
    }
}

TEST_CASE("Overlap of an axis-aligned box and a sphere", "[overlaps][shape][aabox][sphere]")
{
    SECTION("Overlapping")
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Sphere S({0, 0, 0}, 1);
        REQUIRE(B.Overlaps(S));
    }
    SECTION("Overlapping at a point")
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Sphere S({-1, -1, -1}, 1);
        REQUIRE(!B.Overlaps(S));
    }
    SECTION("Non-overlapping")
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Sphere S({-2, 0, 0}, 1);
        REQUIRE(!B.Overlaps(S));
    }
    SECTION("Sphere inside the box")
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Sphere S({1, 1, 1}, 1);
        REQUIRE(B.Overlaps(S));
    }
    SECTION("Box inside the sphere")
    {
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        const physics::Sphere S({1, 1, 1}, 4);
        REQUIRE(B.Overlaps(S));
    }
}

TEST_CASE("Overlap of an axis-aligned box and a oriented box", "[overlaps][shape][aabox][box]")
{
    using math::Vector3;
    const physics::AABox AABox(Vector3(0, 0, 0), Vector3(2, 2, 2));

    SECTION("Box inside AABox")
    {
        const physics::Box Box(Vector3(1, 1, 1), Vector3(0.5, 0.5, 0.5), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(AABox, Box) == true);
    }
    SECTION("Box outside AABox")
    {
        const physics::Box Box(Vector3(4, 4, 4), Vector3(1, 1, 1), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(AABox, Box) == false);
    }
    SECTION("Box partially overlapping AABox")
    {
        const physics::Box Box(Vector3(1, 1, 2.5), Vector3(1, 1, 0.5), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(AABox, Box) == true);
    }
    SECTION("Box touching AABox surface")
    {
        const physics::Box Box(Vector3(3, 1, 1), Vector3(1, 1, 1), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(AABox, Box) == true);
    }
    SECTION("Box rotated and overlapping AABox")
    {
        math::Vector3 XAxis(1, 1, 0);
        math::Vector3 YAxis(-1, 1, 0);
        const math::Vector3 ZAxis(0, 0, 1);
        XAxis = math::Normalize(XAxis);
        YAxis = math::Normalize(YAxis);
        const physics::Box Box(Vector3(1, 1, 1), Vector3(0.5, 0.5, 0.5), XAxis, YAxis, ZAxis);
        REQUIRE(Overlaps(AABox, Box) == true);
    }
}

TEST_CASE("Overlap of a plane and an oriented box", "[overlaps][shape][plane][box]")
{
    using math::Vector3;

    const physics::Plane Plane(Vector3(0, 0, 1), 0);

    SECTION("Box inside Plane")
    {
        const physics::Box Box(Vector3(0, 0, 0), Vector3(0.5, 0.5, 0.5), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(Plane, Box) == true);
    }
    SECTION("Box completely above Plane")
    {
        const physics::Box Box(Vector3(0, 0, 3), Vector3(1, 1, 1), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(Plane, Box) == false);
    }
    SECTION("Box completely below Plane")
    {
        const physics::Box Box(Vector3(0, 0, -3), Vector3(1, 1, 1), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(Plane, Box) == false);
    }
    SECTION("Box partially overlapping Plane")
    {
        const physics::Box Box(Vector3(0, 0, 0.5), Vector3(1, 1, 0.5), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(Plane, Box) == true);
    }
    SECTION("Box rotated and overlapping Plane")
    {
        math::Vector3 XAxis(1, 1, 0);
        math::Vector3 YAxis(-1, 1, 0);
        const math::Vector3 ZAxis(0, 0, 1);
        XAxis = math::Normalize(XAxis);
        YAxis = Normalize(YAxis);
        const physics::Box Box(Vector3(1, 1, 0.5), Vector3(0.5, 0.5, 0.5), XAxis, YAxis, ZAxis);
        REQUIRE(Overlaps(Plane, Box) == true);
    }
}

TEST_CASE("Overlap of a sphere and a oriented box", "[overlaps][shape][sphere][box]")
{
    using math::Vector3;
    const physics::Sphere Sphere(Vector3(0, 0, 0), 2);

    SECTION("Box inside Sphere")
    {
        const physics::Box Box(Vector3(0, 0, 0), Vector3(0.5, 0.5, 0.5), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(Sphere, Box) == true);
    }
    SECTION("Box outside Sphere")
    {
        const physics::Box Box(Vector3(4, 4, 4), Vector3(1, 1, 1), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(Sphere, Box) == false);
    }
    SECTION("Box partially overlapping Sphere")
    {
        const physics::Box Box(Vector3(1, 1, 2.5), Vector3(1, 1, 1), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(Sphere, Box) == true);
    }
    SECTION("Box touching Sphere surface")
    {
        const physics::Box Box(Vector3(3, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(Sphere, Box) == true);
    }
    SECTION("Box rotated and overlapping Sphere")
    {
        math::Vector3 XAxis(1, 1, 0);
        math::Vector3 YAxis(-1, 1, 0);
        const math::Vector3 ZAxis(0, 0, 1);
        XAxis = math::Normalize(XAxis);
        YAxis = math::Normalize(YAxis);
        const physics::Box Box(Vector3(1, 1, 1), Vector3(0.5, 0.5, 0.5), XAxis, YAxis, ZAxis);
        REQUIRE(Overlaps(Sphere, Box) == true);
    }
}

TEST_CASE("Distance between point and a plane", "[distance][shape][plane]")
{
    SECTION("Point on a plane")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Plane P2({0, 0, 1}, 0);
        REQUIRE(physics::Distance(P, P2) == 0);
    }
    SECTION("Point behind a plane")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Plane P2({0, 0, 1}, 1);
        REQUIRE(physics::Distance(P, P2) == -1);
    }
    SECTION("Point in front of the plane")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Plane P2({0, 0, 1}, -1);
        REQUIRE(physics::Distance(P, P2) == 1);
    }
    SECTION("Point behind the plane but the plane is pointing along negative z-axis")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Plane P2({0, 0, -1}, 1);
        REQUIRE(physics::Distance(P, P2) == -1);
    }
}

TEST_CASE("Distance between a point and an axis-aligned box", "[distance][shape][aabox]")
{
    SECTION("Point on the surface ofa a box")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        REQUIRE(physics::Distance(P, B) == 0);
    }
    SECTION("Point closest to the vertex in voronoi region")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({1, 1, 1}, {2, 2, 2});
        REQUIRE(physics::Distance(P, B) == math::Sqrt(3));
    }
    SECTION("Point inside the box")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({-1, -1, -1}, {2, 2, 2});
        REQUIRE(physics::Distance(P, B) == 0);
    }
    SECTION("Point closest to the face/edge")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({1, -2, -2}, {2, 2, 2});
        REQUIRE(physics::Distance(P, B) == 1);
    }
}

TEST_CASE("Distance between a point and a sphere", "[distance][shape][sphere]")
{
    SECTION("Point inside sphere")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Sphere S({0, 0, 0}, 1);
        REQUIRE(physics::Distance(P, S) == 0);
    }
    SECTION("Point outside of the sphere")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Sphere S({1, 1, 1}, 1);
        REQUIRE(physics::Distance(P, S) == (math::Sqrt(3) - 1));
    }
    SECTION("Point on a surface of the sphere")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Sphere S({1, 1, 1}, 2);
        REQUIRE(physics::Distance(P, S) == 0);
    }
}

TEST_CASE("Distance between point and an oriented box", "[distance][shape][box]")
{
    using math::Vector3;
    const physics::Box B1(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                          Vector3(0, 0, 1));
    const physics::real Epsilon = PHYSICS_REALC(1e-6);

    SECTION("Point inside the Box")
    {
        const Vector3 P(0.5, 0.5, 0.5);
        const physics::real R = Distance(P, B1);
        REQUIRE(math::IsEqual(R, 0.0, Epsilon));
    }
    SECTION("Point outside the Box - closest to a vertex")
    {
        const Vector3 P(3, 3, 3);
        const physics::real R = Distance(P, B1);
        REQUIRE(math::IsEqual(R, 2 * math::Sqrt(3.0), Epsilon));
    }
    SECTION("Point outside the Box - closest to an edge")
    {
        const Vector3 P(0.5, 0.5, 2);
        const physics::real R = Distance(P, B1);
        REQUIRE(math::IsEqual(R, 1.0, Epsilon));
    }
    SECTION("Point outside the Box - closest to a face")
    {
        const Vector3 P(2, 0.5, 0.5);
        const physics::real R = Distance(P, B1);
        REQUIRE(math::IsEqual(R, 1.0, Epsilon));
    }
    SECTION("Point on the Box surface")
    {
        const Vector3 P(1, 0.5, 0.5);
        const physics::real R = Distance(P, B1);
        REQUIRE(math::IsEqual(R, 0.0, Epsilon));
    }
    SECTION("Point on the Box edge")
    {
        const Vector3 P(1, 1, 0.5);
        const physics::real R = Distance(P, B1);
        REQUIRE(math::IsEqual(R, 0.0, Epsilon));
    }
    SECTION("Point on the Box vertex")
    {
        const Vector3 P(1, 1, 1);
        const physics::real R = Distance(P, B1);
        REQUIRE(math::IsEqual(R, 0.0, Epsilon));
    }
}

TEST_CASE("Closest point on a plane to the given point", "[closestpoint][shape][plane]")
{
    SECTION("Point on a plane")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Plane P2({0, 0, 1}, 0);
        REQUIRE(physics::ClosestPoint(P, P2) == math::Vector3(0, 0, 0));
    }
    SECTION("Point behind the plane")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Plane P2({0, 0, 1}, 1);
        REQUIRE(physics::ClosestPoint(P, P2) == math::Vector3(0, 0, 1));
    }
    SECTION("Point in front of the plane")
    {
        const math::Vector3 P(0, 2, 0);
        const physics::Plane P2({0, 0, 1}, -1);
        REQUIRE(physics::ClosestPoint(P, P2) == math::Vector3(0, 2, -1));
    }
    SECTION("Point behind the plane and plane normal is along negative z-axis")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Plane P2({0, 0, -1}, 1);
        REQUIRE(physics::ClosestPoint(P, P2) == math::Vector3(0, 0, -1));
    }
}

TEST_CASE("Closest point on an axis-aligned box to the given point", "[closestpoint][shape][aabox]")
{
    SECTION("Point on the surface")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({0, 0, 0}, {2, 2, 2});
        REQUIRE(physics::ClosestPoint(P, B) == math::Vector3(0, 0, 0));
    }
    SECTION("Point closest to the vertex")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({1, 1, 1}, {2, 2, 2});
        REQUIRE(physics::ClosestPoint(P, B) == math::Vector3(1, 1, 1));
    }
    SECTION("Point inside the box")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({-1, -1, -1}, {2, 2, 2});
        REQUIRE(physics::ClosestPoint(P, B) == math::Vector3(0, 0, 0));
    }
    SECTION("Point closest to the face/edge")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::AABox B({1, -5, -5}, {5, 5, 5});
        REQUIRE(physics::ClosestPoint(P, B) == math::Vector3(1, 0, 0));
    }
}

TEST_CASE("Closest point on a sphere to the given point", "[closestpoint][shape][sphere]")
{
    SECTION("Point inside the sphere")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Sphere S({0, 0, 0}, 1);
        REQUIRE(physics::ClosestPoint(P, S) == math::Vector3(0, 0, 0));
    }
    SECTION("Point on the surface")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Sphere S({1, 0, 0}, 1);
        REQUIRE(physics::ClosestPoint(P, S) == math::Vector3(0, 0, 0));
    }
    SECTION("Point outside the sphere")
    {
        const math::Vector3 P(0, 0, 0);
        const physics::Sphere S({-2, 0, 0}, 1);
        REQUIRE(physics::ClosestPoint(P, S) == math::Vector3(-1, 0, 0));
    }
}

TEST_CASE("Closest point on a oriented box to the given point", "[closestpoint][shape][box]")
{
    using math::Vector3;
    const physics::Box Box1(Vector3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                            Vector3(0, 0, 1));

    SECTION("Point inside the Box")
    {
        const Vector3 Point(0.5, 0.5, 0.5);
        const Vector3 Result = ClosestPoint(Point, Box1);
        REQUIRE(Result == Point);
    }
    SECTION("Point outside the Box - closest to a vertex")
    {
        const Vector3 Point(3, 3, 3);
        const Vector3 Result = ClosestPoint(Point, Box1);
        REQUIRE(Result == Vector3(1, 1, 1));
    }
    SECTION("Point outside the Box - closest to an edge")
    {
        const Vector3 Point(0.5, 1, 2);
        const Vector3 Result = ClosestPoint(Point, Box1);
        REQUIRE(Result == Vector3(0.5, 1, 1));
    }
    SECTION("Point outside the Box - closest to a face")
    {
        const Vector3 Point(2, 0.5, 0.5);
        const Vector3 Result = ClosestPoint(Point, Box1);
        REQUIRE(Result == Vector3(1, 0.5, 0.5));
    }
    SECTION("Point on the Box surface")
    {
        const Vector3 Point(1, 0.5, 0.5);
        const Vector3 Result = ClosestPoint(Point, Box1);
        REQUIRE(Result == Point);
    }
    SECTION("Point on the Box edge")
    {
        const Vector3 Point(1, 1, 0.5);
        const Vector3 Result = ClosestPoint(Point, Box1);
        REQUIRE(Result == Point);
    }
    SECTION("Point on the Box vertex")
    {
        const Vector3 Point(1, 1, 1);
        const Vector3 Result = ClosestPoint(Point, Box1);
        REQUIRE(Result == Point);
    }
}