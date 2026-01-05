#include "catch2-helper.hpp"

#include "physics/gjk.hpp"

using namespace Physics;

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

