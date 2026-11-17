#include <wmtk/components/simwild/TightSealGeom.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <functional>

using namespace wmtk;
using namespace wmtk::components::simwild;

// tight_seal::level_crossing, which places both of tight_seal_geom's level sets on an edge: the
// dilation surface where d(x, R) = d/2 (traced from below) and the join region's boundary where
// d(x, S) = d/2 + tolerance (traced from above).
TEST_CASE("tight_seal_level_crossing", "[simwild][tight_seal_geom]")
{
    // 1-Lipschitz: the distance to a point
    const Vector3d c(0.3, -0.2, 0.1);
    const std::function<double(const Vector3d&)> f = [&](const Vector3d& p) {
        return (p - c).norm();
    };
    const Vector3d a(0, 0, 0);
    const Vector3d b(4, 1, -2);

    SECTION("from below and from above, onto the level")
    {
        for (const double step_floor : {1e-3, 0.05, 0.4}) {
            for (const double level : {0.9, 1.7, 2.5}) {
                const auto up = tight_seal::level_crossing(f, a, b, level, step_floor, true);
                REQUIRE(up.has_value());
                CHECK(std::abs(f(*up) - level) < 1e-12);
                const auto down = tight_seal::level_crossing(f, b, a, level, step_floor, false);
                REQUIRE(down.has_value());
                CHECK(std::abs(f(*down) - level) < 1e-12);
            }
        }
    }

    SECTION("the first crossing from the start")
    {
        // the segment passes the point, so the distance falls, then rises: from below, two
        // crossings of a level above its minimum; the one nearer the start is taken
        const Vector3d p(-2, -0.2, 0.1);
        const Vector3d q(3, -0.2, 0.1);
        const auto down = tight_seal::level_crossing(f, p, q, 1.0, 1e-3, false);
        REQUIRE(down.has_value());
        CHECK(std::abs(f(*down) - 1.0) < 1e-12);
        CHECK((*down - p).norm() < (c - p).norm());
    }

    SECTION("nothing from the wrong side or without a crossing")
    {
        CHECK_FALSE(tight_seal::level_crossing(f, a, b, 0.1, 1e-3, true).has_value());
        CHECK_FALSE(tight_seal::level_crossing(f, b, a, 100.0, 1e-3, false).has_value());
        CHECK_FALSE(tight_seal::level_crossing(f, a, b, 100.0, 1e-3, true).has_value());
    }
}
