#include "NumpyCompat.hpp"

#include <cfloat>
#include <cmath>

namespace wmtk::components::polyfem_ops {

namespace {

/// numpy's block size: above this many elements the reduction splits instead of unrolling.
constexpr int64_t PW_BLOCKSIZE = 128;

} // namespace

double pairwise_sum(const double* a, int64_t n)
{
    if (n < 8) {
        double res = 0.0;
        for (int64_t i = 0; i < n; ++i) {
            res += a[i];
        }
        return res;
    }
    if (n <= PW_BLOCKSIZE) {
        // Eight accumulators, seeded with the first eight values, then advanced eight at a time.
        // numpy's comment: the unroll reduces the effective block size to 16 and lets the loop
        // vectorize without changing the summation order.
        double r[8];
        for (int k = 0; k < 8; ++k) {
            r[k] = a[k];
        }
        int64_t i = 8;
        const int64_t end = n - (n % 8);
        for (; i < end; i += 8) {
            for (int k = 0; k < 8; ++k) {
                r[k] += a[i + k];
            }
        }
        double res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
        for (; i < n; ++i) {
            res += a[i];
        }
        return res;
    }
    // Split at half, rounded down to a multiple of the unroll factor so both halves take the
    // unrolled path with the same phase.
    int64_t n2 = n / 2;
    n2 -= n2 % 8;
    return pairwise_sum(a, n2) + pairwise_sum(a + n2, n - n2);
}

double pairwise_sum(const std::vector<double>& a)
{
    return pairwise_sum(a.data(), static_cast<int64_t>(a.size()));
}

double numpy_norm(const double* v, int64_t n)
{
    // Named temporaries: the products must round before they are added, as a non-FMA dot does.
    double acc = 0.0;
    for (int64_t i = 0; i < n; ++i) {
        const double p = v[i] * v[i];
        acc = acc + p;
    }
    return std::sqrt(acc);
}

double numpy_det3(const double rows[3][3])
{
    // LAPACK dgetf2 on a copy: partial pivoting, reciprocal scaling, rank-1 update with an FMA.
    double a[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            a[i][j] = rows[i][j];
        }
    }
    double sign = 1.0;
    for (int j = 0; j < 3; ++j) {
        // idamax: the FIRST row of largest absolute value, ties to the smaller index.
        int p = j;
        for (int i = j + 1; i < 3; ++i) {
            if (std::fabs(a[i][j]) > std::fabs(a[p][j])) {
                p = i;
            }
        }
        if (a[p][j] == 0.0) {
            // dgetf2 reports info > 0 on an exactly zero pivot; numpy then takes sign = 0 and
            // logdet = -inf, so det is 0 * 0 = 0. A degenerate (zero-volume) cell lands here.
            return 0.0;
        }
        if (p != j) {
            for (int k = 0; k < 3; ++k) {
                std::swap(a[p][k], a[j][k]);
            }
            sign = -sign;
        }
        // dgetf2 scales by the reciprocal unless the pivot is below the safe minimum, where the
        // reciprocal would overflow; then it divides. Mirrored for the whole range of the input.
        if (std::fabs(a[j][j]) >= DBL_MIN) {
            const double d = 1.0 / a[j][j];
            for (int i = j + 1; i < 3; ++i) {
                a[i][j] = a[i][j] * d;
            }
        } else {
            for (int i = j + 1; i < 3; ++i) {
                a[i][j] = a[i][j] / a[j][j];
            }
        }
        for (int i = j + 1; i < 3; ++i) {
            for (int k = j + 1; k < 3; ++k) {
                a[i][k] = std::fma(-a[i][j], a[j][k], a[i][k]);
            }
        }
    }
    // numpy's det_from_factored_diagonal: the sign flips on every negative diagonal entry and the
    // logs accumulate left to right, then det = sign * exp(logdet).
    double logdet = 0.0;
    for (int i = 0; i < 3; ++i) {
        double v = a[i][i];
        if (v < 0.0) {
            sign = -sign;
            v = -v;
        }
        logdet = logdet + std::log(v);
    }
    return sign * std::exp(logdet);
}

bool numpy_isclose(const double a, const double b, const double rtol, const double atol)
{
    return std::abs(a - b) <= atol + rtol * std::abs(b);
}

} // namespace wmtk::components::polyfem_ops
