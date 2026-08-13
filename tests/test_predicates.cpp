#include <catch2/catch_test_macros.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/utils/predicates.hpp>

#include <random>

using namespace wmtk;
using namespace wmtk::utils::predicates;

/**
 * The sign convention of the exact predicates, pinned.
 *
 * These wrappers exist so that the exact-predicate backend can be swapped in one file. The
 * hazard in doing that is silent: two libraries can both be exactly correct and still
 * disagree on the SIGN, because they take the determinant of different differences.
 * Shewchuk's orient3d is det[a-d; b-d; c-d], Indirect_Predicates' is det[b-a; c-a; d-a], and
 * those are opposite -- so a backend swap that misses it inverts every tet-orientation test
 * in the toolkit while still passing every test that only asks whether something is
 * degenerate.
 *
 * The expected values below are not derived from the current backend. They were recorded
 * from igl::predicates, the backend in use before the swap, so they pin the behaviour the
 * rest of the toolkit was written against rather than whatever the current backend happens
 * to return.
 */
TEST_CASE("predicate_sign_convention", "[predicates]")
{
    SECTION("orient2d is positive for counter-clockwise")
    {
        CHECK(orient2d(Vector2d(0, 0), Vector2d(1, 0), Vector2d(0, 1)) == Orientation::POSITIVE);
        CHECK(orient2d(Vector2d(0, 0), Vector2d(0, 1), Vector2d(1, 0)) == Orientation::NEGATIVE);
        CHECK(orient2d(Vector2d(0, 0), Vector2d(1, 1), Vector2d(2, 2)) == Orientation::COLLINEAR);
    }

    SECTION("orient3d follows Shewchuk, not the raw backend")
    {
        // The unit simplex in index order is NEGATIVE in this convention. It is POSITIVE
        // under Indirect_Predicates' own orient3d, which is exactly the trap.
        CHECK(
            orient3d(Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 1, 0), Vector3d(0, 0, 1)) ==
            Orientation::NEGATIVE);
        // Swapping two vertices flips it.
        CHECK(
            orient3d(Vector3d(0, 0, 0), Vector3d(0, 1, 0), Vector3d(1, 0, 0), Vector3d(0, 0, 1)) ==
            Orientation::POSITIVE);
        CHECK(
            orient3d(Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 1, 0), Vector3d(1, 1, 0)) ==
            Orientation::COPLANAR);
    }

    SECTION("tet_is_inverted agrees with orient3d")
    {
        // tet_is_inverted is the toolkit's own reading of the sign, and several thousand
        // operations a second depend on it meaning what it did before.
        const Vector3d a(0, 0, 0), b(1, 0, 0), c(0, 1, 0), d(0, 0, 1);
        CHECK(orient3d(a, b, c, d) == Orientation::NEGATIVE);
        CHECK_FALSE(tet_is_inverted(a, b, c, d)); // negative == a well-oriented tet here
        CHECK(tet_is_inverted(a, c, b, d));
    }
}

/**
 * Antisymmetry and degeneracy over random input.
 *
 * A filtered predicate that falls back to exact arithmetic has two code paths, and the
 * interesting failures live where the filter gives up. Feeding it small integers and halves
 * produces exact ties often enough to exercise that path rather than just the fast one.
 */
TEST_CASE("predicate_exactness", "[predicates]")
{
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> small(-3, 3);
    auto c = [&]() { return double(small(rng)) * 0.5; };

    int degenerate2 = 0, degenerate3 = 0;
    for (int i = 0; i < 20000; ++i) {
        const Vector2d a2(c(), c()), b2(c(), c()), c2(c(), c());
        const Orientation o = orient2d(a2, b2, c2);
        // Swapping any two arguments negates the determinant exactly.
        const Orientation swapped = orient2d(b2, a2, c2);
        CHECK(int(o) == -int(swapped));
        if (o == Orientation::COLLINEAR) ++degenerate2;

        const Vector3d a(c(), c(), c()), b(c(), c(), c()), d(c(), c(), c()), e(c(), c(), c());
        const Orientation o3 = orient3d(a, b, d, e);
        CHECK(int(o3) == -int(orient3d(b, a, d, e)));
        if (o3 == Orientation::COPLANAR) ++degenerate3;
    }
    // If these were zero the exact path was never reached and the test proves little.
    CHECK(degenerate2 > 0);
    CHECK(degenerate3 > 0);
}
