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

