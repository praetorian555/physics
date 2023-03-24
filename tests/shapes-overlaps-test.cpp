#include <catch2/catch_test_macros.hpp>

#include "physics/shapes.h"
#include "physics/shapes-overlaps.h"

TEST_CASE("Overlap of two spheres", "[overlaps][shape][sphere]")
{
    SECTION("Identical spheres")
    {
        const Physics::Sphere s1({0, 0, 0}, 1);
        const Physics::Sphere s2({0, 0, 0}, 1);
        REQUIRE(s1.Overlaps(s2));
    }
    SECTION("Overlapping spheres where center is on the surface of the other sphere")
    {
        const Physics::Sphere s1({0, 0, 0}, 1);
        const Physics::Sphere s2({2, 0, 0}, 2);
        REQUIRE(s1.Overlaps(s2));
    }
    SECTION("Spheres overlapping at one point")
    {
        const Physics::Sphere s1({0, 0, 0}, 1);
        const Physics::Sphere s2({2, 0, 0}, 1);
        REQUIRE(s1.Overlaps(s2));
    }
    SECTION("Spheres overlapping where one sphere is contained in another")
    {
        const Physics::Sphere s1({0, 0, 0}, 2);
        const Physics::Sphere s2({1, 0, 0}, 0.5);
        REQUIRE(s1.Overlaps(s2));
    }
    SECTION("Overlapping spheres where centers are not contained in the other sphere")
    {
        const Physics::Sphere s1({0, 0, 0}, 2);
        const Physics::Sphere s2({1.5, 0, 0}, 1);
        REQUIRE(s1.Overlaps(s2));
    }
    SECTION("Non-overlapping spheres")
    {
        const Physics::Sphere s1({0, 0, 0}, 2);
        const Physics::Sphere s2({3, 0, 0}, 0.5);
        REQUIRE(!s1.Overlaps(s2));
    }
    SECTION("Overlapping zero radius sphere")
    {
        const Physics::Sphere s1({0, 0, 0}, 2);
        const Physics::Sphere s2({0, 0, 0}, 0);
        REQUIRE(s1.Overlaps(s2));
    }
    SECTION("Non-overlapping zero radius sphere")
    {
        const Physics::Sphere s1({0, 0, 0}, 2);
        const Physics::Sphere s2({3, 3, 3}, 0);
        REQUIRE(!s1.Overlaps(s2));
    }
}

TEST_CASE("Overlap of two axis-aligned boxes", "[overlaps][shape][aabox]")
{
    SECTION("Identical boxes")
    {
        const Physics::AABox b1({0, 0, 0}, {1, 1, 1});
        const Physics::AABox b2({0, 0, 0}, {1, 1, 1});
        REQUIRE(b1.Overlaps(b2));
    }
    SECTION("One box contains the other")
    {
        const Physics::AABox b1({0, 0, 0}, {1, 1, 1});
        const Physics::AABox b2({0, 0, 0}, {2, 2, 2});
        REQUIRE(b1.Overlaps(b2));
    }
    SECTION("Overlapping at one vertex")
    {
        const Physics::AABox b1({0, 0, 0}, {1, 1, 1});
        const Physics::AABox b2({1, 1, 1}, {2, 2, 2});
        REQUIRE(b1.Overlaps(b2));
    }
    SECTION("Overlapping at one edge")
    {
        const Physics::AABox b1({0, 0, 0}, {1, 1, 1});
        const Physics::AABox b2({1, 0, 1}, {2, 2, 2});
        REQUIRE(b1.Overlaps(b2));
    }
    SECTION("Overlapping at one face")
    {
        const Physics::AABox b1({0, 0, 0}, {1, 1, 1});
        const Physics::AABox b2({1, 0, 0}, {2, 1, 2});
        REQUIRE(b1.Overlaps(b2));
    }
    SECTION("Overlapping but no center is contained in the other box")
    {
        const Physics::AABox b1({0, 0, 0}, {1, 1, 1});
        const Physics::AABox b2({0.5, 0.5, 0.5}, {2, 2, 2});
        REQUIRE(b1.Overlaps(b2));
    }
    SECTION("Non-overlapping boxes")
    {
        const Physics::AABox b1({0, 0, 0}, {1, 1, 1});
        const Physics::AABox b2({2, 2, 2}, {3, 3, 3});
        REQUIRE(!b1.Overlaps(b2));
    }
    SECTION("Overlapping a point box")
    {
        const Physics::AABox b1({0, 0, 0}, {2, 2, 2});
        const Physics::AABox b2({1, 1, 1}, {1, 1, 1});
        REQUIRE(b1.Overlaps(b2));
    }
    SECTION("Non-overlapping point box")
    {
        const Physics::AABox b1({0, 0, 0}, {2, 2, 2});
        const Physics::AABox b2({3, 3, 3}, {3, 3, 3});
        REQUIRE(!b1.Overlaps(b2));
    }
}

TEST_CASE("Overlap of two planes", "[overlaps][shape][plane]")
{
    SECTION("Parallel planes")
    {
        const Physics::Plane p1({0, 0, 1}, 0);
        const Physics::Plane p2({0, 0, 1}, 3);
        REQUIRE(!p1.Overlaps(p2));
    }
    SECTION("Identical planes")
    {
        const Physics::Plane p1({0, 0, 1}, 0);
        const Physics::Plane p2({0, 0, 1}, 0);
        REQUIRE(p1.Overlaps(p2));
    }
    SECTION("Parallel planes with opposite normals")
    {
        const Physics::Plane p1({0, 0, 1}, 0);
        const Physics::Plane p2({0, 0, -1}, 3);
        REQUIRE(!p1.Overlaps(p2));
    }
    SECTION("Parallel planes with opposite normals but same offsets")
    {
        const Physics::Plane p1({0, 0, 1}, 0);
        const Physics::Plane p2({0, 0, -1}, 0);
        REQUIRE(p1.Overlaps(p2));
    }
    SECTION("Overlapping planes")
    {
        const Physics::Plane p1({0, 0, 1}, 0);
        const Physics::Plane p2({0, 1, 0}, -3);
        REQUIRE(p1.Overlaps(p2));
    }
}

TEST_CASE("Overlap of two oriented boxes", "[overlaps][shape][box]")
{
    using math::Vector3;
    using math::Point3;

    SECTION("Non-overlapping Boxes with same orientation")
    {
        const Physics::Box box1(Point3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const Physics::Box box2(Point3(3, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        REQUIRE(!Overlaps(box1, box2));
    }
    SECTION("Overlapping Boxes with same orientation")
    {
        const Physics::Box box1(Point3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const Physics::Box box2(Point3(0.5, 0.5, 0.5), Vector3(1, 1, 1), Vector3(1, 0, 0),
                                Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(box1, box2));
    }
    SECTION("Overlapping Boxes with different orientations")
    {
        const Physics::Box box1(Point3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const Physics::Box box2(Point3(0, 0, 0), Vector3(1, 1, 1), Vector3(0, 1, 0), Vector3(-1, 0, 0),
                                Vector3(0, 0, 1));
        REQUIRE(Overlaps(box1, box2));
    }
    SECTION("Non-overlapping Boxes with different orientations")
    {
        const Physics::Box box1(Point3(3, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const Physics::Box box2(Point3(0, 0, 0), Vector3(1, 1, 1), Vector3(0, 1, 0), Vector3(-1, 0, 0),
                                Vector3(0, 0, 1));
        REQUIRE(!Overlaps(box1, box2));
    }
    SECTION("Overlapping Boxes with orthogonal orientations")
    {
        const Physics::Box box1(Point3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const Physics::Box box2(Point3(0, 0, 0), Vector3(1, 1, 1), Vector3(0, 0, 1), Vector3(0, 1, 0),
                                Vector3(-1, 0, 0));
        REQUIRE(Overlaps(box1, box2));
    }
    SECTION("Identical Boxes")
    {
        const Physics::Box box1(Point3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        REQUIRE(Overlaps(box1, box1));
    }
    SECTION("Boxes touching at one Point")
    {
        const Physics::Box box1(Point3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const Physics::Box box2(Point3(2, 2, 2), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        REQUIRE(Overlaps(box1, box2));
    }
    SECTION("Boxes touching along an edge")
    {
        const Physics::Box box1(Point3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const Physics::Box box2(Point3(2, 2, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        REQUIRE(Overlaps(box1, box2));
    }
    SECTION("Boxes touching along a face")
    {
        const Physics::Box box1(Point3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const Physics::Box box2(Point3(2, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        REQUIRE(Overlaps(box1, box2));
    }
    SECTION("Overlapping point box")
    {
        const Physics::Box box1(Point3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const Physics::Box box2(Point3(1, 1, 1), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        REQUIRE(Overlaps(box1, box2));
    }
    SECTION("Non-overlapping point box")
    {
        const Physics::Box box1(Point3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        const Physics::Box box2(Point3(2, 2, 2), Vector3(2, 2, 2), Vector3(1, 0, 0), Vector3(0, 1, 0),
                                Vector3(0, 0, 1));
        REQUIRE(Overlaps(box1, box2));
    }
}

TEST_CASE("Overlap of a sphere and a plane", "[overlaps][shape][sphere][plane]")
{
    SECTION("Overlapping")
    {
        const Physics::Sphere s({0, 0, 0}, 1);
        const Physics::Plane p({0, 0, 1}, 0);
        REQUIRE(s.Overlaps(p));
    }
    SECTION("Non-overlapping")
    {
        const Physics::Sphere s({0, 0, 0}, 1);
        const Physics::Plane p({0, 0, 1}, 2);
        REQUIRE(!s.Overlaps(p));
    }
    SECTION("Non-overlapping where plane normal is in negative direction")
    {
        const Physics::Sphere s({0, 0, 0}, 1);
        const Physics::Plane p({0, 0, 1}, -2);
        REQUIRE(!s.Overlaps(p));
    }
    SECTION("Overlapping at one point")
    {
        const Physics::Sphere s({0, 0, 0}, 1);
        const Physics::Plane p({0, 0, 1}, 1);
        REQUIRE(s.Overlaps(p));
    }
    SECTION("Overlapping at one point on the other side of sphere")
    {
        const Physics::Sphere s({0, 0, 0}, 1);
        const Physics::Plane p({0, 0, 1}, -1);
        REQUIRE(s.Overlaps(p));
    }
    SECTION("Overlapping point sphere")
    {
        const Physics::Sphere s({0, 0, 0}, 0);
        const Physics::Plane p({0, 0, 1}, 0);
        REQUIRE(s.Overlaps(p));
    }
    SECTION("Non-overlapping point sphere")
    {
        const Physics::Sphere s({0, 0, 0}, 0);
        const Physics::Plane p({0, 0, 1}, 1);
        REQUIRE(!s.Overlaps(p));
    }
}

TEST_CASE("Overlap of an axis-aligned box and a plane", "[overlaps][shape][aabox][plane]")
{
    SECTION("Overlapping on a face")
    {
        const Physics::AABox b({0, 0, 0}, {2, 2, 2});
        const Physics::Plane p({1, 0, 0}, 0);
        REQUIRE(b.Overlaps(p));
    }
    SECTION("Overlapping on a vertex")
    {
        const Physics::AABox b({0, 0, 0}, {2, 2, 2});
        const Physics::Plane p({1, 1, 1}, 0);
        REQUIRE(b.Overlaps(p));
    }
    SECTION("Overlapping on an edge")
    {
        const Physics::AABox b({0, 0, 0}, {2, 2, 2});
        const Physics::Plane p({1, 1, 0}, 0);
        REQUIRE(b.Overlaps(p));
    }
    SECTION("Overlapping on a face from the negative side of a plane")
    {
        const Physics::AABox b({0, 0, 0}, {2, 2, 2});
        const Physics::Plane p({1, 0, 0}, 2);
        REQUIRE(b.Overlaps(p));
    }
    SECTION("Overlapping")
    {
        const Physics::AABox b({0, 0, 0}, {2, 2, 2});
        const Physics::Plane p({1, 1, 1}, 1);
        REQUIRE(b.Overlaps(p));
    }
    SECTION("Non-overlapping on positive side")
    {
        const Physics::AABox b({0, 0, 0}, {2, 2, 2});
        const Physics::Plane p({1, 1, 1}, -1);
        REQUIRE(!b.Overlaps(p));
    }
    SECTION("Non-overlapping on negative side")
    {
        const Physics::AABox b({0, 0, 0}, {2, 2, 2});
        const Physics::Plane p({1, 1, 1}, 5);
        REQUIRE(!b.Overlaps(p));
    }
    SECTION("Overlapping point box")
    {
        const Physics::AABox b({0, 0, 0}, {0, 0, 0});
        const Physics::Plane p({1, 0, 0}, 0);
        REQUIRE(b.Overlaps(p));
    }
    SECTION("Non-overlapping point box")
    {
        const Physics::AABox b({0, 0, 0}, {0, 0, 0});
        const Physics::Plane p({1, 0, 0}, 1);
        REQUIRE(!b.Overlaps(p));
    }
}

TEST_CASE("Overlap of an axis-aligned box and a sphere", "[overlaps][shape][aabox][sphere]")
{
    SECTION("Overlapping")
    {
        const Physics::AABox b({0, 0, 0}, {2, 2, 2});
        const Physics::Sphere s({0, 0, 0}, 1);
        REQUIRE(b.Overlaps(s));
    }
    SECTION("Overlapping at a point")
    {
        const Physics::AABox b({0, 0, 0}, {2, 2, 2});
        const Physics::Sphere s({-1, -1, -1}, 1);
        REQUIRE(!b.Overlaps(s));
    }
    SECTION("Non-overlapping")
    {
        const Physics::AABox b({0, 0, 0}, {2, 2, 2});
        const Physics::Sphere s({-2, 0, 0}, 1);
        REQUIRE(!b.Overlaps(s));
    }
    SECTION("Sphere inside the box")
    {
        const Physics::AABox b({0, 0, 0}, {2, 2, 2});
        const Physics::Sphere s({1, 1, 1}, 1);
        REQUIRE(b.Overlaps(s));
    }
    SECTION("Box inside the sphere")
    {
        const Physics::AABox b({0, 0, 0}, {2, 2, 2});
        const Physics::Sphere s({1, 1, 1}, 4);
        REQUIRE(b.Overlaps(s));
    }
}

TEST_CASE("Overlap of an axis-aligned box and a oriented box", "[overlaps][shape][aabox][box]")
{
    using math::Vector3;
    using math::Point3;
    const Physics::AABox aa_box(Point3(0, 0, 0), Point3(2, 2, 2));

    SECTION("Box inside AABox")
    {
        const Physics::Box box(Point3(1, 1, 1), Vector3(0.5, 0.5, 0.5), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(aa_box, box) == true);
    }
    SECTION("Box outside AABox")
    {
        const Physics::Box box(Point3(4, 4, 4), Vector3(1, 1, 1), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(aa_box, box) == false);
    }
    SECTION("Box partially overlapping AABox")
    {
        const Physics::Box box(Point3(1, 1, 2.5), Vector3(1, 1, 0.5), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(aa_box, box) == true);
    }
    SECTION("Box touching AABox surface")
    {
        const Physics::Box box(Point3(3, 1, 1), Vector3(1, 1, 1), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(aa_box, box) == true);
    }
    SECTION("Box rotated and overlapping AABox")
    {
        math::Vector3 axis_x(1, 1, 0);
        math::Vector3 axis_y(-1, 1, 0);
        const math::Vector3 axis_z(0, 0, 1);
        axis_x = math::Normalize(axis_x);
        axis_y = math::Normalize(axis_y);
        const Physics::Box box(Point3(1, 1, 1), Vector3(0.5, 0.5, 0.5), axis_x, axis_y, axis_z);
        REQUIRE(Overlaps(aa_box, box) == true);
    }
}

TEST_CASE("Overlap of a plane and an oriented box", "[overlaps][shape][plane][box]")
{
    using math::Vector3;
    using math::Point3;

    const Physics::Plane plane(Vector3(0, 0, 1), 0);

    SECTION("Box inside Plane")
    {
        const Physics::Box box(Point3(0, 0, 0), Vector3(0.5, 0.5, 0.5), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(plane, box) == true);
    }
    SECTION("Box completely above Plane")
    {
        const Physics::Box box(Point3(0, 0, 3), Vector3(1, 1, 1), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(plane, box) == false);
    }
    SECTION("Box completely below Plane")
    {
        const Physics::Box box(Point3(0, 0, -3), Vector3(1, 1, 1), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(plane, box) == false);
    }
    SECTION("Box partially overlapping Plane")
    {
        const Physics::Box box(Point3(0, 0, 0.5), Vector3(1, 1, 0.5), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(plane, box) == true);
    }
    SECTION("Box rotated and overlapping Plane")
    {
        math::Vector3 axis_x(1, 1, 0);
        math::Vector3 axis_y(-1, 1, 0);
        const math::Vector3 axis_z(0, 0, 1);
        axis_x = math::Normalize(axis_x);
        axis_y = Normalize(axis_y);
        const Physics::Box box(Point3(1, 1, 0.5), Vector3(0.5, 0.5, 0.5), axis_x, axis_y, axis_z);
        REQUIRE(Overlaps(plane, box) == true);
    }
}

TEST_CASE("Overlap of a sphere and a oriented box", "[overlaps][shape][sphere][box]")
{
    using math::Vector3;
    using math::Point3;
    const Physics::Sphere sphere(Point3(0, 0, 0), 2);

    SECTION("Box inside Sphere")
    {
        const Physics::Box box(Point3(0, 0, 0), Vector3(0.5, 0.5, 0.5), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(sphere, box) == true);
    }
    SECTION("Box outside Sphere")
    {
        const Physics::Box box(Point3(4, 4, 4), Vector3(1, 1, 1), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(sphere, box) == false);
    }
    SECTION("Box partially overlapping Sphere")
    {
        const Physics::Box box(Point3(1, 1, 2.5), Vector3(1, 1, 1), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(sphere, box) == true);
    }
    SECTION("Box touching Sphere surface")
    {
        const Physics::Box box(Point3(3, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0),
                               Vector3(0, 1, 0), Vector3(0, 0, 1));
        REQUIRE(Overlaps(sphere, box) == true);
    }
    SECTION("Box rotated and overlapping Sphere")
    {
        math::Vector3 axis_x(1, 1, 0);
        math::Vector3 axis_y(-1, 1, 0);
        const math::Vector3 axis_z(0, 0, 1);
        axis_x = math::Normalize(axis_x);
        axis_y = math::Normalize(axis_y);
        const Physics::Box box(Point3(1, 1, 1), Vector3(0.5, 0.5, 0.5), axis_x, axis_y, axis_z);
        REQUIRE(Overlaps(sphere, box) == true);
    }
}
