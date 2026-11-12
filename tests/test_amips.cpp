#include <wmtk/utils/AMIPS.h>
#include <wmtk/utils/Rational.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>

// AMIPS_energy_p3_certified must return exactly what the GMP path returns, whenever it returns
// anything: AMIPS_energy_stable_p3 substitutes it for AMIPS_energy_rational_p3 on the tets the
// double energy cannot handle, and tetwild's output depends on those values bit for bit.
//
// Seven families: random tets, nearly flat ones (the fallback's real population) at three
// offsets and scales, tets on a small integer grid (the energy is often a dyadic rational or an
// exact double -- uncertifiable, so the GMP path must take them), exactly coplanar dyadic tets
// (flat: +inf), and tets with coincident points.
TEST_CASE("amips_certified_fallback_matches_gmp", "[amips]")
{
    const auto same_bits = [](double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; };
    std::mt19937_64 rng(12345);
    std::uniform_real_distribution<double> U(-1, 1);
    constexpr long n_per_family = 20000;
    long certified = 0, total = 0, mismatches = 0;
    for (int family = 0; family < 7; ++family) {
        for (long it = 0; it < n_per_family; ++it) {
            std::array<double, 12> T;
            const double offset = (family == 2 || family == 3) ? 137.25 : 0.0;
            const double scale = family == 3 ? 1e-6 : 1.0;
            for (int i = 0; i < 12; ++i) T[i] = offset + scale * U(rng);
            if (family >= 1 && family <= 3) {
                // p3 just off the plane of p0, p1, p2
                const double u = U(rng), v = U(rng);
                const double eps = std::ldexp(U(rng), -int(20 + rng() % 40));
                for (int k = 0; k < 3; ++k) {
                    T[9 + k] = T[k] + u * (T[3 + k] - T[k]) + v * (T[6 + k] - T[k]) + eps * scale;
                }
            } else if (family == 4) {
                for (int i = 0; i < 12; ++i) T[i] = double(int64_t(rng() % 7) - 3);
            } else if (family == 5) {
                for (int i = 0; i < 9; ++i) {
                    T[i] = std::ldexp(double(int64_t(rng() % 2001) - 1000), -10);
                }
                for (int k = 0; k < 3; ++k) T[9 + k] = T[3 + k] + T[6 + k] - T[k];
            } else if (family == 6) {
                for (int k = 0; k < 3; ++k) T[3 + k] = T[k];
            }
            double fast;
            if (!wmtk::AMIPS_energy_p3_certified(T, fast)) continue;
            ++certified;
            const double exact = wmtk::AMIPS_energy_rational_p3<wmtk::Rational>(T);
            if (!same_bits(fast, exact)) {
                ++mismatches;
                UNSCOPED_INFO("family " << family << ": certified " << fast << ", GMP " << exact);
            }
        }
        total += n_per_family;
    }
    CHECK(mismatches == 0);
    // It must also certify most of them, or it is not doing its job.
    CHECK(certified > total / 2);
}
