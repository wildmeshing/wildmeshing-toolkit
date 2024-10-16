#include <catch2/catch_test_macros.hpp>
#include <wmtk/utils/edge_insertion.hpp>

using namespace wmtk::utils;
using namespace wmtk;

TEST_CASE("test_segment_intersection", "[edgeinsertion][.]")
{
    SECTION("general intersection")
    {
        Vector2r P1(0, 0);
        Vector2r P2(2, 2);
        Vector2r Q1(0, 2);
        Vector2r Q2(2, 0);
        Vector2r intersection;

        CHECK(segment_intersection_rational(P1, P2, Q1, Q2, intersection));
        CHECK(intersection[0] == Rational(1));
        CHECK(intersection[1] == Rational(1));

        CHECK(segment_intersection_rational(P1, P2, Q2, Q1, intersection));
        CHECK(intersection[0] == Rational(1));
        CHECK(intersection[1] == Rational(1));

        CHECK(segment_intersection_rational(P2, P1, Q1, Q2, intersection));
        CHECK(intersection[0] == Rational(1));
        CHECK(intersection[1] == Rational(1));

        CHECK(segment_intersection_rational(P2, P1, Q2, Q1, intersection));
        CHECK(intersection[0] == Rational(1));
        CHECK(intersection[1] == Rational(1));
    }
    SECTION("intersect at endpoint")
    {
        Vector2r intersection;

        CHECK(segment_intersection_rational(
            Vector2r(0, 0),
            Vector2r(2, 2),
            Vector2r(1, 1),
            Vector2r(2, 0),
            intersection));
        CHECK(intersection[0] == Rational(1));
        CHECK(intersection[1] == Rational(1));

        CHECK(segment_intersection_rational(
            Vector2r(0, 0),
            Vector2r(2, 2),
            Vector2r(1, 1),
            Vector2r(0, 2),
            intersection));
        CHECK(intersection[0] == Rational(1));
        CHECK(intersection[1] == Rational(1));

        CHECK(segment_intersection_rational(
            Vector2r(0, 0),
            Vector2r(2, 2),
            Vector2r(0, 0),
            Vector2r(1, -1),
            intersection));
        CHECK(intersection[0] == Rational(0));
        CHECK(intersection[1] == Rational(0));


        CHECK(segment_intersection_rational(
            Vector2r(0, 0),
            Vector2r(2, 2),
            Vector2r(0, 0),
            Vector2r(-1, 1),
            intersection));
        CHECK(intersection[0] == Rational(0));
        CHECK(intersection[1] == Rational(0));


        CHECK(segment_intersection_rational(
            Vector2r(0, 0),
            Vector2r(2, 2),
            Vector2r(3, 1),
            Vector2r(2, 2),
            intersection));
        CHECK(intersection[0] == Rational(2));
        CHECK(intersection[1] == Rational(2));

        CHECK(segment_intersection_rational(
            Vector2r(0, 0),
            Vector2r(2, 2),
            Vector2r(1, 3),
            Vector2r(2, 2),
            intersection));
        CHECK(intersection[0] == Rational(2));
        CHECK(intersection[1] == Rational(2));

        CHECK(segment_intersection_rational(
            Vector2r(0, 0),
            Vector2r(2, 2),
            Vector2r(0, 4),
            Vector2r(4, 0),
            intersection));
        CHECK(intersection[0] == Rational(2));
        CHECK(intersection[1] == Rational(2));

        CHECK(segment_intersection_rational(
            Vector2r(0, 0),
            Vector2r(2, 2),
            Vector2r(1, -2),
            Vector2r(-1, 2),
            intersection));
        CHECK(intersection[0] == Rational(0));
        CHECK(intersection[1] == Rational(0));

        CHECK(segment_intersection_rational(
            Vector2r(0, 0),
            Vector2r(2, 2),
            Vector2r(2, 2),
            Vector2r(3, 3),
            intersection));
        CHECK(intersection[0] == Rational(2));
        CHECK(intersection[1] == Rational(2));

        CHECK(segment_intersection_rational(
            Vector2r(0, 0),
            Vector2r(2, 2),
            Vector2r(-1, -1),
            Vector2r(0, 0),
            intersection));
        CHECK(intersection[0] == Rational(0));
        CHECK(intersection[1] == Rational(0));

        CHECK(segment_intersection_rational(
            Vector2r(0, 0),
            Vector2r(2, 2),
            Vector2r(1, 1),
            Vector2r(3, 3),
            intersection));
        CHECK(intersection[0] == Rational(1));
        CHECK(intersection[1] == Rational(1));

        CHECK(segment_intersection_rational(
            Vector2r(0, 0),
            Vector2r(2, 2),
            Vector2r(1.5, 1.5),
            Vector2r(1, 1),
            intersection));
        CHECK(intersection[0] == Rational(1.5));
        CHECK(intersection[1] == Rational(1.5));
    }
    SECTION("no intersection")
    {
        Vector2r intersection;

        CHECK(!segment_intersection_rational(
            Vector2r(0, 0),
            Vector2r(1, 1),
            Vector2r(2, 2),
            Vector2r(3, 3),
            intersection));
        CHECK(!segment_intersection_rational(
            Vector2r(0, 0),
            Vector2r(1, 1),
            Vector2r(2, 2),
            Vector2r(3, 0),
            intersection));
    }
}