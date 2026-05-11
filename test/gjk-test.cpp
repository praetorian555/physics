#include "catch2-helper.hpp"

#include "physics/gjk.hpp"
#include "physics/shapes/box-shape.hpp"
#include "physics/shapes/sphere-shape.hpp"

using namespace Physics;

namespace
{
Body MakeSphereBody(SphereShape& shape, const Vector3r& position)
{
    Body body;
    body.position = position;
    body.orientation = Quatr::Identity();
    body.linear_velocity = Vector3r::Zero();
    body.angular_velocity = Vector3r::Zero();
    body.inverse_mass = 1.0f;
    body.elasticity = 1.0f;
    body.friction = 0.0f;
    body.shape = &shape;
    return body;
}

Opal::DynamicArray<Vector3r> MakeBoxVertices(const Vector3r& size)
{
    return Opal::DynamicArray<Vector3r>{
        Vector3r(-size.x / 2, -size.y / 2, -size.z / 2), Vector3r(size.x / 2, -size.y / 2, -size.z / 2),
        Vector3r(-size.x / 2, size.y / 2, -size.z / 2),  Vector3r(size.x / 2, size.y / 2, -size.z / 2),
        Vector3r(-size.x / 2, -size.y / 2, size.z / 2),  Vector3r(size.x / 2, -size.y / 2, size.z / 2),
        Vector3r(-size.x / 2, size.y / 2, size.z / 2),   Vector3r(size.x / 2, size.y / 2, size.z / 2),
    };
}

Body MakeBoxBody(BoxShape& shape, const Vector3r& position, const Quatr& orientation = Quatr::Identity())
{
    Body body;
    body.position = position;
    body.orientation = orientation;
    body.linear_velocity = Vector3r::Zero();
    body.angular_velocity = Vector3r::Zero();
    body.inverse_mass = 1.0f;
    body.elasticity = 1.0f;
    body.friction = 0.0f;
    body.shape = &shape;
    return body;
}
}  // namespace

TEST_CASE("SignedVolume1D - Origin projects to middle of segment", "[GJK]")
{
    const Vector3r start(-1.0f, 0.0f, 0.0f);
    const Vector3r end(1.0f, 0.0f, 0.0f);
    const Vector2r result = SignedVolume1D(start, end);
    REQUIRE(result[0] == Catch::Approx(0.5f));
    REQUIRE(result[1] == Catch::Approx(0.5f));
}

TEST_CASE("SignedVolume1D - Origin projects closer to start", "[GJK]")
{
    const Vector3r start(-1.0f, 0.0f, 0.0f);
    const Vector3r end(3.0f, 0.0f, 0.0f);
    const Vector2r result = SignedVolume1D(start, end);
    REQUIRE(result[0] == Catch::Approx(0.75f));
    REQUIRE(result[1] == Catch::Approx(0.25f));
}

TEST_CASE("SignedVolume1D - Origin projects closer to end", "[GJK]")
{
    const Vector3r start(-3.0f, 0.0f, 0.0f);
    const Vector3r end(1.0f, 0.0f, 0.0f);
    const Vector2r result = SignedVolume1D(start, end);
    REQUIRE(result[0] == Catch::Approx(0.25f));
    REQUIRE(result[1] == Catch::Approx(0.75f));
}

TEST_CASE("SignedVolume1D - Origin projects beyond start point", "[GJK]")
{
    const Vector3r start(1.0f, 0.0f, 0.0f);
    const Vector3r end(3.0f, 0.0f, 0.0f);
    const Vector2r result = SignedVolume1D(start, end);
    REQUIRE(result[0] == Catch::Approx(1.0f));
    REQUIRE(result[1] == Catch::Approx(0.0f));
}

TEST_CASE("SignedVolume1D - Origin projects beyond end point", "[GJK]")
{
    const Vector3r start(-3.0f, 0.0f, 0.0f);
    const Vector3r end(-1.0f, 0.0f, 0.0f);
    const Vector2r result = SignedVolume1D(start, end);
    REQUIRE(result[0] == Catch::Approx(0.0f));
    REQUIRE(result[1] == Catch::Approx(1.0f));
}

TEST_CASE("SignedVolume1D - Segment along Y axis", "[GJK]")
{
    const Vector3r start(0.0f, -2.0f, 0.0f);
    const Vector3r end(0.0f, 2.0f, 0.0f);
    const Vector2r result = SignedVolume1D(start, end);
    REQUIRE(result[0] == Catch::Approx(0.5f));
    REQUIRE(result[1] == Catch::Approx(0.5f));
}

TEST_CASE("SignedVolume1D - Segment along Z axis", "[GJK]")
{
    const Vector3r start(0.0f, 0.0f, -2.0f);
    const Vector3r end(0.0f, 0.0f, 2.0f);
    const Vector2r result = SignedVolume1D(start, end);
    REQUIRE(result[0] == Catch::Approx(0.5f));
    REQUIRE(result[1] == Catch::Approx(0.5f));
}

TEST_CASE("SignedVolume1D - Diagonal segment in XY plane", "[GJK]")
{
    const Vector3r start(-1.0f, -1.0f, 0.0f);
    const Vector3r end(1.0f, 1.0f, 0.0f);
    const Vector2r result = SignedVolume1D(start, end);
    REQUIRE(result[0] == Catch::Approx(0.5f));
    REQUIRE(result[1] == Catch::Approx(0.5f));
}

TEST_CASE("SignedVolume1D - Origin perpendicular to segment", "[GJK]")
{
    const Vector3r start(-1.0f, 1.0f, 0.0f);
    const Vector3r end(1.0f, 1.0f, 0.0f);
    const Vector2r result = SignedVolume1D(start, end);
    REQUIRE(result[0] == Catch::Approx(0.5f));
    REQUIRE(result[1] == Catch::Approx(0.5f));
}

TEST_CASE("SignedVolume1D - Origin on segment start", "[GJK]")
{
    const Vector3r start(0.0f, 0.0f, 0.0f);
    const Vector3r end(2.0f, 0.0f, 0.0f);
    const Vector2r result = SignedVolume1D(start, end);
    REQUIRE(result[0] == Catch::Approx(1.0f));
    REQUIRE(result[1] == Catch::Approx(0.0f));
}

TEST_CASE("SignedVolume1D - Origin on segment end", "[GJK]")
{
    const Vector3r start(-2.0f, 0.0f, 0.0f);
    const Vector3r end(0.0f, 0.0f, 0.0f);
    const Vector2r result = SignedVolume1D(start, end);
    REQUIRE(result[0] == Catch::Approx(0.0f));
    REQUIRE(result[1] == Catch::Approx(1.0f));
}

TEST_CASE("SignedVolume1D - 3D diagonal segment", "[GJK]")
{
    const Vector3r start(-1.0f, -1.0f, -1.0f);
    const Vector3r end(1.0f, 1.0f, 1.0f);
    const Vector2r result = SignedVolume1D(start, end);
    REQUIRE(result[0] == Catch::Approx(0.5f));
    REQUIRE(result[1] == Catch::Approx(0.5f));
}

TEST_CASE("SignedVolume1D - Barycentric coordinates sum to 1", "[GJK]")
{
    const Vector3r start(-3.0f, 2.0f, 1.0f);
    const Vector3r end(2.0f, -1.0f, 4.0f);
    const Vector2r result = SignedVolume1D(start, end);
    REQUIRE(result[0] + result[1] == Catch::Approx(1.0f));
}

TEST_CASE("SignedVolume2D - Origin inside triangle at centroid", "[GJK]")
{
    const Vector3r a(-1.0f, -1.0f, 0.0f);
    const Vector3r b(1.0f, -1.0f, 0.0f);
    const Vector3r c(0.0f, 1.0f, 0.0f);
    const Vector3r result = SignedVolume2D(a, b, c);
    REQUIRE(result[0] + result[1] + result[2] == Catch::Approx(1.0f));
    REQUIRE(result[0] == Catch::Approx(0.25f));
    REQUIRE(result[1] == Catch::Approx(0.25f));
    REQUIRE(result[2] == Catch::Approx(0.5f));
}

TEST_CASE("SignedVolume2D - Origin at vertex a", "[GJK]")
{
    const Vector3r a(0.0f, 0.0f, 0.0f);
    const Vector3r b(2.0f, 0.0f, 0.0f);
    const Vector3r c(1.0f, 2.0f, 0.0f);
    const Vector3r result = SignedVolume2D(a, b, c);
    REQUIRE(result[0] == Catch::Approx(1.0f));
    REQUIRE(result[1] == Catch::Approx(0.0f));
    REQUIRE(result[2] == Catch::Approx(0.0f));
}

TEST_CASE("SignedVolume2D - Origin at vertex b", "[GJK]")
{
    const Vector3r a(-2.0f, 0.0f, 0.0f);
    const Vector3r b(0.0f, 0.0f, 0.0f);
    const Vector3r c(-1.0f, 2.0f, 0.0f);
    const Vector3r result = SignedVolume2D(a, b, c);
    REQUIRE(result[0] == Catch::Approx(0.0f));
    REQUIRE(result[1] == Catch::Approx(1.0f));
    REQUIRE(result[2] == Catch::Approx(0.0f));
}

TEST_CASE("SignedVolume2D - Origin at vertex c", "[GJK]")
{
    const Vector3r a(-1.0f, -2.0f, 0.0f);
    const Vector3r b(1.0f, -2.0f, 0.0f);
    const Vector3r c(0.0f, 0.0f, 0.0f);
    const Vector3r result = SignedVolume2D(a, b, c);
    REQUIRE(result[0] == Catch::Approx(0.0f));
    REQUIRE(result[1] == Catch::Approx(0.0f));
    REQUIRE(result[2] == Catch::Approx(1.0f));
}

TEST_CASE("SignedVolume2D - Origin closest to edge bc", "[GJK]")
{
    const Vector3r a(0.0f, 5.0f, 0.0f);
    const Vector3r b(-1.0f, 0.0f, 0.0f);
    const Vector3r c(1.0f, 0.0f, 0.0f);
    const Vector3r result = SignedVolume2D(a, b, c);
    REQUIRE(result[0] == Catch::Approx(0.0f));
    REQUIRE(result[1] == Catch::Approx(0.5f));
    REQUIRE(result[2] == Catch::Approx(0.5f));
}

TEST_CASE("SignedVolume2D - Origin closest to edge ca", "[GJK]")
{
    const Vector3r a(-1.0f, 1.0f, 0.0f);
    const Vector3r b(0.0f, 5.0f, 0.0f);
    const Vector3r c(1.0f, 1.0f, 0.0f);
    const Vector3r result = SignedVolume2D(a, b, c);
    REQUIRE(result[0] == Catch::Approx(0.5f));
    REQUIRE(result[1] == Catch::Approx(0.0f));
    REQUIRE(result[2] == Catch::Approx(0.5f));
}

TEST_CASE("SignedVolume2D - Origin closest to edge ab", "[GJK]")
{
    const Vector3r a(-1.0f, 0.0f, 0.0f);
    const Vector3r b(1.0f, 0.0f, 0.0f);
    const Vector3r c(0.0f, -5.0f, 0.0f);
    const Vector3r result = SignedVolume2D(a, b, c);
    REQUIRE(result[0] == Catch::Approx(0.5f));
    REQUIRE(result[1] == Catch::Approx(0.5f));
    REQUIRE(result[2] == Catch::Approx(0.0f));
}

TEST_CASE("SignedVolume2D - Triangle in YZ plane", "[GJK]")
{
    const Vector3r a(0.0f, -1.0f, -1.0f);
    const Vector3r b(0.0f, 1.0f, -1.0f);
    const Vector3r c(0.0f, 0.0f, 1.0f);
    const Vector3r result = SignedVolume2D(a, b, c);
    REQUIRE(result[0] + result[1] + result[2] == Catch::Approx(1.0f));
    REQUIRE(result[0] > 0.0f);
    REQUIRE(result[1] > 0.0f);
    REQUIRE(result[2] > 0.0f);
}

TEST_CASE("SignedVolume2D - Triangle in XZ plane", "[GJK]")
{
    const Vector3r a(-1.0f, 0.0f, -1.0f);
    const Vector3r b(1.0f, 0.0f, -1.0f);
    const Vector3r c(0.0f, 0.0f, 1.0f);
    const Vector3r result = SignedVolume2D(a, b, c);
    REQUIRE(result[0] + result[1] + result[2] == Catch::Approx(1.0f));
    REQUIRE(result[0] > 0.0f);
    REQUIRE(result[1] > 0.0f);
    REQUIRE(result[2] > 0.0f);
}

TEST_CASE("SignedVolume2D - Origin above triangle plane", "[GJK]")
{
    const Vector3r a(-1.0f, -1.0f, -2.0f);
    const Vector3r b(1.0f, -1.0f, -2.0f);
    const Vector3r c(0.0f, 1.0f, -2.0f);
    const Vector3r result = SignedVolume2D(a, b, c);
    REQUIRE(result[0] + result[1] + result[2] == Catch::Approx(1.0f));
    REQUIRE(result[0] > 0.0f);
    REQUIRE(result[1] > 0.0f);
    REQUIRE(result[2] > 0.0f);
}

TEST_CASE("SignedVolume2D - 3D triangle not axis-aligned", "[GJK]")
{
    const Vector3r a(-1.0f, -1.0f, -1.0f);
    const Vector3r b(1.0f, -1.0f, 1.0f);
    const Vector3r c(0.0f, 1.0f, 0.0f);
    const Vector3r result = SignedVolume2D(a, b, c);
    REQUIRE(result[0] + result[1] + result[2] == Catch::Approx(1.0f));
}

TEST_CASE("SignedVolume2D - Origin outside closest to vertex", "[GJK]")
{
    const Vector3r a(1.0f, 1.0f, 0.0f);
    const Vector3r b(3.0f, 1.0f, 0.0f);
    const Vector3r c(2.0f, 3.0f, 0.0f);
    const Vector3r result = SignedVolume2D(a, b, c);
    REQUIRE(result[0] == Catch::Approx(1.0f));
    REQUIRE(result[1] == Catch::Approx(0.0f));
    REQUIRE(result[2] == Catch::Approx(0.0f));
}

TEST_CASE("SignedVolume2D - Barycentric coordinates sum to 1", "[GJK]")
{
    const Vector3r a(-2.0f, 1.0f, 3.0f);
    const Vector3r b(1.0f, -2.0f, 1.0f);
    const Vector3r c(3.0f, 2.0f, -1.0f);
    const Vector3r result = SignedVolume2D(a, b, c);
    REQUIRE(result[0] + result[1] + result[2] == Catch::Approx(1.0f));
}

TEST_CASE("SignedVolume2D - Reconstructed point is closest to origin", "[GJK]")
{
    const Vector3r a(-1.0f, -1.0f, 0.0f);
    const Vector3r b(1.0f, -1.0f, 0.0f);
    const Vector3r c(0.0f, 1.0f, 0.0f);
    const Vector3r result = SignedVolume2D(a, b, c);
    const Vector3r closest = a * result[0] + b * result[1] + c * result[2];
    REQUIRE(closest[0] == Catch::Approx(0.0f));
    REQUIRE(closest[1] == Catch::Approx(0.0f));
    REQUIRE(closest[2] == Catch::Approx(0.0f));
}

TEST_CASE("SignedVolume3D - Origin inside tetrahedron", "[GJK]")
{
    const Vector3r a(1.0f, -1.0f, -1.0f);
    const Vector3r b(-1.0f, -1.0f, -1.0f);
    const Vector3r c(0.0f, 1.0f, 0.0f);
    const Vector3r d(0.0f, -1.0f, 1.0f);
    const Vector4r result = SignedVolume3D(a, b, c, d);
    REQUIRE(result[0] + result[1] + result[2] + result[3] == Catch::Approx(1.0f));
    REQUIRE(result[0] > 0.0f);
    REQUIRE(result[1] > 0.0f);
    REQUIRE(result[2] > 0.0f);
    REQUIRE(result[3] > 0.0f);
}

TEST_CASE("SignedVolume3D - Origin at vertex a", "[GJK]")
{
    const Vector3r a(0.0f, 0.0f, 0.0f);
    const Vector3r b(2.0f, 0.0f, 0.0f);
    const Vector3r c(1.0f, 2.0f, 0.0f);
    const Vector3r d(1.0f, 1.0f, 2.0f);
    const Vector4r result = SignedVolume3D(a, b, c, d);
    REQUIRE(result[0] == Catch::Approx(1.0f));
    REQUIRE(result[1] == Catch::Approx(0.0f));
    REQUIRE(result[2] == Catch::Approx(0.0f));
    REQUIRE(result[3] == Catch::Approx(0.0f));
}

TEST_CASE("SignedVolume3D - Origin at vertex b", "[GJK]")
{
    const Vector3r a(-2.0f, 0.0f, 0.0f);
    const Vector3r b(0.0f, 0.0f, 0.0f);
    const Vector3r c(-1.0f, 2.0f, 0.0f);
    const Vector3r d(-1.0f, 1.0f, 2.0f);
    const Vector4r result = SignedVolume3D(a, b, c, d);
    REQUIRE(result[0] == Catch::Approx(0.0f));
    REQUIRE(result[1] == Catch::Approx(1.0f));
    REQUIRE(result[2] == Catch::Approx(0.0f));
    REQUIRE(result[3] == Catch::Approx(0.0f));
}

TEST_CASE("SignedVolume3D - Origin at vertex c", "[GJK]")
{
    const Vector3r a(-1.0f, -2.0f, 0.0f);
    const Vector3r b(1.0f, -2.0f, 0.0f);
    const Vector3r c(0.0f, 0.0f, 0.0f);
    const Vector3r d(0.0f, -1.0f, 2.0f);
    const Vector4r result = SignedVolume3D(a, b, c, d);
    REQUIRE(result[0] == Catch::Approx(0.0f));
    REQUIRE(result[1] == Catch::Approx(0.0f));
    REQUIRE(result[2] == Catch::Approx(1.0f));
    REQUIRE(result[3] == Catch::Approx(0.0f));
}

TEST_CASE("SignedVolume3D - Origin at vertex d", "[GJK]")
{
    const Vector3r a(-1.0f, -1.0f, -2.0f);
    const Vector3r b(1.0f, -1.0f, -2.0f);
    const Vector3r c(0.0f, 1.0f, -2.0f);
    const Vector3r d(0.0f, 0.0f, 0.0f);
    const Vector4r result = SignedVolume3D(a, b, c, d);
    REQUIRE(result[0] == Catch::Approx(0.0f));
    REQUIRE(result[1] == Catch::Approx(0.0f));
    REQUIRE(result[2] == Catch::Approx(0.0f));
    REQUIRE(result[3] == Catch::Approx(1.0f));
}

TEST_CASE("SignedVolume3D - Origin closest to face abc", "[GJK]")
{
    const Vector3r a(-1.0f, -1.0f, 0.0f);
    const Vector3r b(1.0f, -1.0f, 0.0f);
    const Vector3r c(0.0f, 1.0f, 0.0f);
    const Vector3r d(0.0f, 0.0f, -5.0f);
    const Vector4r result = SignedVolume3D(a, b, c, d);
    REQUIRE(result[0] + result[1] + result[2] + result[3] == Catch::Approx(1.0f));
    REQUIRE(result[0] > 0.0f);
    REQUIRE(result[1] > 0.0f);
    REQUIRE(result[2] > 0.0f);
    REQUIRE(result[3] == Catch::Approx(0.0f));
}

TEST_CASE("SignedVolume3D - Origin closest to face bcd", "[GJK]")
{
    const Vector3r a(0.0f, 0.0f, 10.0f);
    const Vector3r b(2.0f, 0.0f, 2.0f);
    const Vector3r c(0.0f, 2.0f, 2.0f);
    const Vector3r d(-1.0f, -1.0f, 2.0f);
    const Vector4r result = SignedVolume3D(a, b, c, d);
    REQUIRE(result[0] + result[1] + result[2] + result[3] == Catch::Approx(1.0f));
    REQUIRE(result[0] == Catch::Approx(0.0f));
}

TEST_CASE("SignedVolume3D - Origin outside closest to edge", "[GJK]")
{
    const Vector3r a(-1.0f, 0.0f, 5.0f);
    const Vector3r b(1.0f, 0.0f, 5.0f);
    const Vector3r c(0.0f, 5.0f, 5.0f);
    const Vector3r d(0.0f, 0.0f, 10.0f);
    const Vector4r result = SignedVolume3D(a, b, c, d);
    REQUIRE(result[0] + result[1] + result[2] + result[3] == Catch::Approx(1.0f));
    REQUIRE(result[0] == Catch::Approx(0.5f));
    REQUIRE(result[1] == Catch::Approx(0.5f));
    REQUIRE(result[2] == Catch::Approx(0.0f));
    REQUIRE(result[3] == Catch::Approx(0.0f));
}

TEST_CASE("SignedVolume3D - Origin outside closest to vertex", "[GJK]")
{
    const Vector3r a(1.0f, 1.0f, 1.0f);
    const Vector3r b(3.0f, 1.0f, 1.0f);
    const Vector3r c(2.0f, 3.0f, 1.0f);
    const Vector3r d(2.0f, 2.0f, 3.0f);
    const Vector4r result = SignedVolume3D(a, b, c, d);
    REQUIRE(result[0] == Catch::Approx(1.0f));
    REQUIRE(result[1] == Catch::Approx(0.0f));
    REQUIRE(result[2] == Catch::Approx(0.0f));
    REQUIRE(result[3] == Catch::Approx(0.0f));
}

TEST_CASE("SignedVolume3D - Barycentric coordinates sum to 1", "[GJK]")
{
    const Vector3r a(-2.0f, 1.0f, 3.0f);
    const Vector3r b(1.0f, -2.0f, 1.0f);
    const Vector3r c(3.0f, 2.0f, -1.0f);
    const Vector3r d(0.0f, 3.0f, 2.0f);
    const Vector4r result = SignedVolume3D(a, b, c, d);
    REQUIRE(result[0] + result[1] + result[2] + result[3] == Catch::Approx(1.0f));
}

TEST_CASE("SignedVolume3D - Reconstructed point is closest to origin when inside", "[GJK]")
{
    const Vector3r a(1.0f, 0.0f, -1.0f);
    const Vector3r b(-1.0f, 0.0f, -1.0f);
    const Vector3r c(0.0f, 1.0f, -1.0f);
    const Vector3r d(0.0f, 0.0f, 1.0f);
    const Vector4r result = SignedVolume3D(a, b, c, d);
    const Vector3r closest = a * result[0] + b * result[1] + c * result[2] + d * result[3];
    REQUIRE(closest[0] == Catch::Approx(0.0f));
    REQUIRE(closest[1] == Catch::Approx(0.0f));
    REQUIRE(closest[2] == Catch::Approx(0.0f));
}

TEST_CASE("SignedVolume3D - Symmetric tetrahedron with origin at centroid", "[GJK]")
{
    const Vector3r a(1.0f, 1.0f, 1.0f);
    const Vector3r b(-1.0f, -1.0f, 1.0f);
    const Vector3r c(-1.0f, 1.0f, -1.0f);
    const Vector3r d(1.0f, -1.0f, -1.0f);
    const Vector4r result = SignedVolume3D(a, b, c, d);
    REQUIRE(result[0] == Catch::Approx(0.25f));
    REQUIRE(result[1] == Catch::Approx(0.25f));
    REQUIRE(result[2] == Catch::Approx(0.25f));
    REQUIRE(result[3] == Catch::Approx(0.25f));
}

TEST_CASE("IntersectGJK - Coincident spheres intersect", "[GJK]")
{
    SphereShape shape_a(1.0f);
    SphereShape shape_b(1.0f);
    const Body body_a = MakeSphereBody(shape_a, Vector3r(0, 0, 0));
    const Body body_b = MakeSphereBody(shape_b, Vector3r(0, 0, 0));
    REQUIRE(IntersectGJK(body_a, body_b));
}

TEST_CASE("IntersectGJK - Overlapping spheres intersect", "[GJK]")
{
    SphereShape shape_a(1.0f);
    SphereShape shape_b(1.0f);
    const Body body_a = MakeSphereBody(shape_a, Vector3r(0, 0, 0));
    const Body body_b = MakeSphereBody(shape_b, Vector3r(1.5f, 0, 0));
    REQUIRE(IntersectGJK(body_a, body_b));
}

TEST_CASE("IntersectGJK - Disjoint spheres do not intersect", "[GJK]")
{
    SphereShape shape_a(1.0f);
    SphereShape shape_b(1.0f);
    const Body body_a = MakeSphereBody(shape_a, Vector3r(0, 0, 0));
    const Body body_b = MakeSphereBody(shape_b, Vector3r(5, 0, 0));
    REQUIRE_FALSE(IntersectGJK(body_a, body_b));
}

TEST_CASE("IntersectGJK - Spheres separated along Y axis do not intersect", "[GJK]")
{
    SphereShape shape_a(0.5f);
    SphereShape shape_b(0.5f);
    const Body body_a = MakeSphereBody(shape_a, Vector3r(0, 0, 0));
    const Body body_b = MakeSphereBody(shape_b, Vector3r(0, 3, 0));
    REQUIRE_FALSE(IntersectGJK(body_a, body_b));
}

TEST_CASE("IntersectGJK - Spheres separated along Z axis do not intersect", "[GJK]")
{
    SphereShape shape_a(0.5f);
    SphereShape shape_b(0.5f);
    const Body body_a = MakeSphereBody(shape_a, Vector3r(0, 0, 0));
    const Body body_b = MakeSphereBody(shape_b, Vector3r(0, 0, 3));
    REQUIRE_FALSE(IntersectGJK(body_a, body_b));
}

TEST_CASE("IntersectGJK - Sphere fully contained inside larger sphere intersects", "[GJK]")
{
    SphereShape shape_a(5.0f);
    SphereShape shape_b(0.5f);
    const Body body_a = MakeSphereBody(shape_a, Vector3r(0, 0, 0));
    const Body body_b = MakeSphereBody(shape_b, Vector3r(1, 1, 1));
    REQUIRE(IntersectGJK(body_a, body_b));
}

TEST_CASE("IntersectGJK - Sphere and box overlap", "[GJK]")
{
    SphereShape sphere(1.0f);
    BoxShape box;
    auto vertices = MakeBoxVertices(Vector3r(2, 2, 2));
    box.Build(Opal::ArrayView<Vector3r>(vertices.GetData(), vertices.GetSize()));
    const Body body_a = MakeSphereBody(sphere, Vector3r(1.5f, 0, 0));
    const Body body_b = MakeBoxBody(box, Vector3r(0, 0, 0));
    REQUIRE(IntersectGJK(body_a, body_b));
}

TEST_CASE("IntersectGJK - Sphere and box disjoint", "[GJK]")
{
    SphereShape sphere(0.5f);
    BoxShape box;
    auto vertices = MakeBoxVertices(Vector3r(2, 2, 2));
    box.Build(Opal::ArrayView<Vector3r>(vertices.GetData(), vertices.GetSize()));
    const Body body_a = MakeSphereBody(sphere, Vector3r(5, 0, 0));
    const Body body_b = MakeBoxBody(box, Vector3r(0, 0, 0));
    REQUIRE_FALSE(IntersectGJK(body_a, body_b));
}

TEST_CASE("IntersectGJK - Overlapping boxes intersect", "[GJK]")
{
    BoxShape box_a;
    BoxShape box_b;
    auto vertices_a = MakeBoxVertices(Vector3r(2, 2, 2));
    auto vertices_b = MakeBoxVertices(Vector3r(2, 2, 2));
    box_a.Build(Opal::ArrayView<Vector3r>(vertices_a.GetData(), vertices_a.GetSize()));
    box_b.Build(Opal::ArrayView<Vector3r>(vertices_b.GetData(), vertices_b.GetSize()));
    const Body body_a = MakeBoxBody(box_a, Vector3r(0, 0, 0));
    const Body body_b = MakeBoxBody(box_b, Vector3r(1.5f, 0, 0));
    REQUIRE(IntersectGJK(body_a, body_b));
}

TEST_CASE("IntersectGJK - Disjoint boxes do not intersect", "[GJK]")
{
    BoxShape box_a;
    BoxShape box_b;
    auto vertices_a = MakeBoxVertices(Vector3r(2, 2, 2));
    auto vertices_b = MakeBoxVertices(Vector3r(2, 2, 2));
    box_a.Build(Opal::ArrayView<Vector3r>(vertices_a.GetData(), vertices_a.GetSize()));
    box_b.Build(Opal::ArrayView<Vector3r>(vertices_b.GetData(), vertices_b.GetSize()));
    const Body body_a = MakeBoxBody(box_a, Vector3r(0, 0, 0));
    const Body body_b = MakeBoxBody(box_b, Vector3r(5, 0, 0));
    REQUIRE_FALSE(IntersectGJK(body_a, body_b));
}

TEST_CASE("IntersectGJK - Rotated box overlaps neighbor", "[GJK]")
{
    BoxShape box_a;
    BoxShape box_b;
    auto vertices_a = MakeBoxVertices(Vector3r(2, 2, 2));
    auto vertices_b = MakeBoxVertices(Vector3r(2, 2, 2));
    box_a.Build(Opal::ArrayView<Vector3r>(vertices_a.GetData(), vertices_a.GetSize()));
    box_b.Build(Opal::ArrayView<Vector3r>(vertices_b.GetData(), vertices_b.GetSize()));
    const Quatr rotation = Quatr::FromAxisAngleRadians(Vector3r(0, 0, 1), Opal::k_pi_float / 4.0f);
    const Body body_a = MakeBoxBody(box_a, Vector3r(0, 0, 0), rotation);
    const Body body_b = MakeBoxBody(box_b, Vector3r(2.2f, 0, 0));
    REQUIRE(IntersectGJK(body_a, body_b));
}

TEST_CASE("IntersectGJK - Rotated box clears axis-aligned neighbor", "[GJK]")
{
    BoxShape box_a;
    BoxShape box_b;
    auto vertices_a = MakeBoxVertices(Vector3r(2, 2, 2));
    auto vertices_b = MakeBoxVertices(Vector3r(2, 2, 2));
    box_a.Build(Opal::ArrayView<Vector3r>(vertices_a.GetData(), vertices_a.GetSize()));
    box_b.Build(Opal::ArrayView<Vector3r>(vertices_b.GetData(), vertices_b.GetSize()));
    const Quatr rotation = Quatr::FromAxisAngleRadians(Vector3r(0, 0, 1), Opal::k_pi_float / 4.0f);
    const Body body_a = MakeBoxBody(box_a, Vector3r(0, 0, 0), rotation);
    const Body body_b = MakeBoxBody(box_b, Vector3r(6.0f, 0, 0));
    REQUIRE_FALSE(IntersectGJK(body_a, body_b));
}
