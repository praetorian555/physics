#include <catch2/catch_test_macros.hpp>

#include "physics/shapes-helpers.h"
#include "physics/shapes.h"

TEST_CASE("Distance between point and a plane", "[distance][shape][plane]")
{
    SECTION("Point on a plane")
    {
        const math::Point3 p(0, 0, 0);
        const physics::Plane p2({0, 0, 1}, 0);
        REQUIRE(physics::Distance(p, p2) == 0);
    }
    SECTION("Point behind a plane")
    {
        const math::Point3 p(0, 0, 0);
        const physics::Plane p2({0, 0, 1}, 1);
        REQUIRE(physics::Distance(p, p2) == -1);
    }
    SECTION("Point in front of the plane")
    {
        const math::Point3 p(0, 0, 0);
        const physics::Plane p2({0, 0, 1}, -1);
        REQUIRE(physics::Distance(p, p2) == 1);
    }
    SECTION("Point behind the plane but the plane is pointing along negative z-axis")
    {
        const math::Point3 p(0, 0, 0);
        const physics::Plane p2({0, 0, -1}, 1);
        REQUIRE(physics::Distance(p, p2) == -1);
    }
}

TEST_CASE("Distance between a point and an axis-aligned box", "[distance][shape][aabox]")
{
    SECTION("Point on the surface ofa a box")
    {
        const math::Point3 p(0, 0, 0);
        const physics::AABox b({0, 0, 0}, {2, 2, 2});
        REQUIRE(physics::Distance(p, b) == 0);
    }
    SECTION("Point closest to the vertex in voronoi region")
    {
        const math::Point3 p(0, 0, 0);
        const physics::AABox b({1, 1, 1}, {2, 2, 2});
        REQUIRE(physics::Distance(p, b) == math::Sqrt(3));
    }
    SECTION("Point inside the box")
    {
        const math::Point3 p(0, 0, 0);
        const physics::AABox b({-1, -1, -1}, {2, 2, 2});
        REQUIRE(physics::Distance(p, b) == 0);
    }
    SECTION("Point closest to the face/edge")
    {
        const math::Point3 p(0, 0, 0);
        const physics::AABox b({1, -2, -2}, {2, 2, 2});
        REQUIRE(physics::Distance(p, b) == 1);
    }
}

TEST_CASE("Distance between a point and a sphere", "[distance][shape][sphere]")
{
    SECTION("Point inside sphere")
    {
        const math::Point3 p(0, 0, 0);
        const physics::Sphere s({0, 0, 0}, 1);
        REQUIRE(physics::Distance(p, s) == 0);
    }
    SECTION("Point outside of the sphere")
    {
        const math::Point3 p(0, 0, 0);
        const physics::Sphere s({1, 1, 1}, 1);
        REQUIRE(physics::Distance(p, s) == (math::Sqrt(3) - 1));
    }
    SECTION("Point on a surface of the sphere")
    {
        const math::Point3 p(0, 0, 0);
        const physics::Sphere s({1, 1, 1}, 2);
        REQUIRE(physics::Distance(p, s) == 0);
    }
}

TEST_CASE("Distance between point and an oriented box", "[distance][shape][box]")
{
    using math::Point3;
    using math::Vector3;
    const physics::Box b1(Point3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                          Vector3(0, 0, 1));
    constexpr physics::real k_epsilon = PHYSICS_REALC(1e-6);

    SECTION("Point inside the Box")
    {
        const Point3 p(0.5, 0.5, 0.5);
        const physics::real r = Distance(p, b1);
        REQUIRE(math::IsEqual(r, 0.0, k_epsilon));
    }
    SECTION("Point outside the Box - closest to a vertex")
    {
        const Point3 p(3, 3, 3);
        const physics::real r = Distance(p, b1);
        REQUIRE(math::IsEqual(r, 2 * math::Sqrt(3.0), k_epsilon));
    }
    SECTION("Point outside the Box - closest to an edge")
    {
        const Point3 p(0.5, 0.5, 2);
        const physics::real r = Distance(p, b1);
        REQUIRE(math::IsEqual(r, 1.0, k_epsilon));
    }
    SECTION("Point outside the Box - closest to a face")
    {
        const Point3 p(2, 0.5, 0.5);
        const physics::real r = Distance(p, b1);
        REQUIRE(math::IsEqual(r, 1.0, k_epsilon));
    }
    SECTION("Point on the Box surface")
    {
        const Point3 p(1, 0.5, 0.5);
        const physics::real r = Distance(p, b1);
        REQUIRE(math::IsEqual(r, 0.0, k_epsilon));
    }
    SECTION("Point on the Box edge")
    {
        const Point3 p(1, 1, 0.5);
        const physics::real r = Distance(p, b1);
        REQUIRE(math::IsEqual(r, 0.0, k_epsilon));
    }
    SECTION("Point on the Box vertex")
    {
        const Point3 p(1, 1, 1);
        const physics::real r = Distance(p, b1);
        REQUIRE(math::IsEqual(r, 0.0, k_epsilon));
    }
}

TEST_CASE("Closest point on a plane to the given point", "[closestpoint][shape][plane]")
{
    SECTION("Point on a plane")
    {
        const math::Point3 p(0, 0, 0);
        const physics::Plane p2({0, 0, 1}, 0);
        REQUIRE(physics::ClosestPoint(p, p2) == math::Point3(0, 0, 0));
    }
    SECTION("Point behind the plane")
    {
        const math::Point3 p(0, 0, 0);
        const physics::Plane p2({0, 0, 1}, 1);
        REQUIRE(physics::ClosestPoint(p, p2) == math::Point3(0, 0, 1));
    }
    SECTION("Point in front of the plane")
    {
        const math::Point3 p(0, 2, 0);
        const physics::Plane p2({0, 0, 1}, -1);
        REQUIRE(physics::ClosestPoint(p, p2) == math::Point3(0, 2, -1));
    }
    SECTION("Point behind the plane and plane normal is along negative z-axis")
    {
        const math::Point3 p(0, 0, 0);
        const physics::Plane p2({0, 0, -1}, 1);
        REQUIRE(physics::ClosestPoint(p, p2) == math::Point3(0, 0, -1));
    }
}

TEST_CASE("Closest point on an axis-aligned box to the given point", "[closestpoint][shape][aabox]")
{
    SECTION("Point on the surface")
    {
        const math::Point3 p(0, 0, 0);
        const physics::AABox b({0, 0, 0}, {2, 2, 2});
        REQUIRE(physics::ClosestPoint(p, b) == math::Point3(0, 0, 0));
    }
    SECTION("Point closest to the vertex")
    {
        const math::Point3 p(0, 0, 0);
        const physics::AABox b({1, 1, 1}, {2, 2, 2});
        REQUIRE(physics::ClosestPoint(p, b) == math::Point3(1, 1, 1));
    }
    SECTION("Point inside the box")
    {
        const math::Point3 p(0, 0, 0);
        const physics::AABox b({-1, -1, -1}, {2, 2, 2});
        REQUIRE(physics::ClosestPoint(p, b) == math::Point3(0, 0, 0));
    }
    SECTION("Point closest to the face/edge")
    {
        const math::Point3 p(0, 0, 0);
        const physics::AABox b({1, -5, -5}, {5, 5, 5});
        REQUIRE(physics::ClosestPoint(p, b) == math::Point3(1, 0, 0));
    }
}

TEST_CASE("Closest point on a sphere to the given point", "[closestpoint][shape][sphere]")
{
    SECTION("Point inside the sphere")
    {
        const math::Point3 p(0, 0, 0);
        const physics::Sphere s({0, 0, 0}, 1);
        REQUIRE(physics::ClosestPoint(p, s) == math::Point3(0, 0, 0));
    }
    SECTION("Point on the surface")
    {
        const math::Point3 p(0, 0, 0);
        const physics::Sphere s({1, 0, 0}, 1);
        REQUIRE(physics::ClosestPoint(p, s) == math::Point3(0, 0, 0));
    }
    SECTION("Point outside the sphere")
    {
        const math::Point3 p(0, 0, 0);
        const physics::Sphere s({-2, 0, 0}, 1);
        REQUIRE(physics::ClosestPoint(p, s) == math::Point3(-1, 0, 0));
    }
}

TEST_CASE("Closest point on a oriented box to the given point", "[closestpoint][shape][box]")
{
    using math::Point3;
    using math::Vector3;
    const physics::Box box1(Point3(0, 0, 0), Vector3(1, 1, 1), Vector3(1, 0, 0), Vector3(0, 1, 0),
                            Vector3(0, 0, 1));

    SECTION("Point inside the Box")
    {
        const Point3 point(0.5, 0.5, 0.5);
        const Point3 result = ClosestPoint(point, box1);
        REQUIRE(result == point);
    }
    SECTION("Point outside the Box - closest to a vertex")
    {
        const Point3 point(3, 3, 3);
        const Point3 result = ClosestPoint(point, box1);
        REQUIRE(result == Point3(1, 1, 1));
    }
    SECTION("Point outside the Box - closest to an edge")
    {
        const Point3 point(0.5, 1, 2);
        const Point3 result = ClosestPoint(point, box1);
        REQUIRE(result == Point3(0.5, 1, 1));
    }
    SECTION("Point outside the Box - closest to a face")
    {
        const Point3 point(2, 0.5, 0.5);
        const Point3 result = ClosestPoint(point, box1);
        REQUIRE(result == Point3(1, 0.5, 0.5));
    }
    SECTION("Point on the Box surface")
    {
        const Point3 point(1, 0.5, 0.5);
        const Point3 result = ClosestPoint(point, box1);
        REQUIRE(result == point);
    }
    SECTION("Point on the Box edge")
    {
        const Point3 point(1, 1, 0.5);
        const Point3 result = ClosestPoint(point, box1);
        REQUIRE(result == point);
    }
    SECTION("Point on the Box vertex")
    {
        const Point3 point(1, 1, 1);
        const Point3 result = ClosestPoint(point, box1);
        REQUIRE(result == point);
    }
}

TEST_CASE("Enclosing of two axis-aligned boxes with another", "[shape][aabox][enclose]")
{
    SECTION("Identical boxes")
    {
        const physics::AABox b1({0, 0, 0}, {1, 1, 1});
        const physics::AABox b2({0, 0, 0}, {1, 1, 1});
        physics::AABox result;
        physics::Enclose(result, b1, b2);
        REQUIRE(result.min == b1.min);
        REQUIRE(result.max == b1.max);
    }
    SECTION("Overlapping boxes")
    {
        const physics::AABox b1({0, 0, 0}, {1, 1, 1});
        const physics::AABox b2({0.5, 0.5, 0.5}, {2, 2, 2});
        physics::AABox result;
        physics::Enclose(result, b1, b2);
        REQUIRE(result.min == b1.min);
        REQUIRE(result.max == b2.max);
    }
    SECTION("Non-overlapping boxes")
    {
        const physics::AABox b1({0, 0, 0}, {1, 1, 1});
        const physics::AABox b2({2, 2, 2}, {3, 3, 3});
        physics::AABox result;
        physics::Enclose(result, b1, b2);
        REQUIRE(result.min == b1.min);
        REQUIRE(result.max == b2.max);
    }
    SECTION("Point box and regular box that are overlapping")
    {
        const physics::AABox b1({0, 0, 0}, {0, 0, 0});
        const physics::AABox b2({0, 0, 0}, {1, 1, 1});
        physics::AABox result;
        physics::Enclose(result, b1, b2);
        REQUIRE(result.min == b1.min);
        REQUIRE(result.max == b2.max);
    }
    SECTION("Point box and regular box that are not overlapping")
    {
        const physics::AABox b1({0, 0, 0}, {0, 0, 0});
        const physics::AABox b2({1, 1, 1}, {2, 2, 2});
        physics::AABox result;
        physics::Enclose(result, b1, b2);
        REQUIRE(result.min == b1.min);
        REQUIRE(result.max == b2.max);
    }
}

TEST_CASE("Enclosing of two spheres", "[shape][sphere][enclose]")
{
    SECTION("Identical spheres")
    {
        const physics::Sphere s1({0, 0, 0}, 1);
        const physics::Sphere s2({0, 0, 0}, 1);
        physics::Sphere result;
        physics::Enclose(result, s1, s2);
        REQUIRE(result.center == s1.center);
        REQUIRE(result.radius == s1.radius);
    }
    SECTION("Identical point spheres")
    {
        const physics::Sphere s1({0, 0, 0}, 0);
        const physics::Sphere s2({0, 0, 0}, 0);
        physics::Sphere result;
        physics::Enclose(result, s1, s2);
        REQUIRE(result.center == s1.center);
        REQUIRE(result.radius == s1.radius);
    }
    SECTION("One sphere in the another")
    {
        const physics::Sphere s1({0, 0, 0}, 1);
        const physics::Sphere s2({0.5, 0, 0}, 0.5);
        physics::Sphere result;
        physics::Enclose(result, s1, s2);
        REQUIRE(result.center == s1.center);
        REQUIRE(result.radius == s1.radius);
    }
    SECTION("One sphere in the another and their centers match")
    {
        const physics::Sphere s1({0, 0, 0}, 1);
        const physics::Sphere s2({0, 0, 0}, 0.5);
        physics::Sphere result;
        physics::Enclose(result, s1, s2);
        REQUIRE(result.center == s1.center);
        REQUIRE(result.radius == s1.radius);
    }
    SECTION("Two sphere overlapping")
    {
        const physics::Sphere s1({0, 0, 0}, 1);
        const physics::Sphere s2({1, 0, 0}, 1);
        physics::Sphere result;
        physics::Enclose(result, s1, s2);
        REQUIRE(result.center == math::Point3(0.5, 0, 0));
        REQUIRE(result.radius == 1.5);
    }
    SECTION("Two sphere not overlapping")
    {
        const physics::Sphere s1({0, 0, 0}, 1);
        const physics::Sphere s2({2, 0, 0}, 1);
        physics::Sphere result;
        physics::Enclose(result, s1, s2);
        REQUIRE(result.center == math::Point3(1, 0, 0));
        REQUIRE(result.radius == 2);
    }
    SECTION("Regular sphere and point sphere overlapping")
    {
        const physics::Sphere s1({0, 0, 0}, 1);
        const physics::Sphere s2({0.5, 0, 0}, 0);
        physics::Sphere result;
        physics::Enclose(result, s1, s2);
        REQUIRE(result.center == s1.center);
        REQUIRE(result.radius == s1.radius);
    }
    SECTION("Regular sphere and point sphere not overlapping")
    {
        const physics::Sphere s1({0, 0, 0}, 1);
        const physics::Sphere s2({2, 0, 0}, 0);
        physics::Sphere result;
        physics::Enclose(result, s1, s2);
        REQUIRE(result.center == math::Point3(0.5, 0, 0));
        REQUIRE(result.radius == 1.5);
    }
}
