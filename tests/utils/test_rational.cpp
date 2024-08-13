#include <catch2/catch_test_macros.hpp>
#include <wmtk/Types.hpp>

using namespace wmtk;

TEST_CASE("test_rational", "[rational]")
{
    Rational r0(1);
    Rational r1(1.0, true);

    CHECK(r0 == r1);
    CHECK(r0.get_sign() == 1);
    CHECK(r1.get_sign() == 1);

    {
        Rational r2(r0);
        Rational r3(r1);

        CHECK(r0 == r2);
        CHECK(r0 == r3);

        CHECK(r1 == r2);
        CHECK(r1 == r3);

        CHECK(r2 == r3);
    }

    {
        Rational r2 = r0 * r0;
        Rational r3 = r0 * r1;
        Rational r4 = r1 * r1;

        CHECK(r0 == r2);
        CHECK(r0 == r3);
        CHECK(r0 == r4);


        CHECK(r1 == r2);
        CHECK(r1 == r3);
        CHECK(r1 == r4);
    }

    {
        Rational r2 = r0 / r0;
        Rational r3 = r0 / r1;
        Rational r4 = r1 / r1;

        CHECK(r0 == r2);
        CHECK(r0 == r3);
        CHECK(r0 == r4);


        CHECK(r1 == r2);
        CHECK(r1 == r3);
        CHECK(r1 == r4);
    }

    {
        Rational r2 = r0 + r0;
        Rational r3 = r0 + r1;
        Rational r4 = r1 + r1;


        Rational r5(2.0);
        Rational r6(2, true);

        CHECK(r5 == r2);
        CHECK(r5 == r3);
        CHECK(r5 == r4);


        CHECK(r6 == r2);
        CHECK(r6 == r3);
        CHECK(r6 == r4);
    }

    {
        Rational r2 = r0 - r0;
        Rational r3 = r0 - r1;
        Rational r4 = r1 - r1;


        Rational r5;
        Rational r6(true);

        CHECK(r5 == r2);
        CHECK(r5 == r3);
        CHECK(r5 == r4);


        CHECK(r6 == r2);
        CHECK(r6 == r3);
        CHECK(r6 == r4);
    }
}