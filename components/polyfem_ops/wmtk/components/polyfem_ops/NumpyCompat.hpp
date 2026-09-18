#pragma once

#include <cstdint>
#include <vector>

namespace wmtk::components::polyfem_ops {

/**
 * @brief Sum a float64 buffer exactly as `np.sum` does.
 *
 * The constraint matrices are normalized by `np.sum(values)` and `np.sum(L_values ** 2)`
 * (constraints.write_fitting_constraint_hdf5 / write_laplacian_constraint_hdf5), and every
 * written value is divided by that number, so a last-bit difference in the sum moves every
 * value. numpy does NOT accumulate a running total: its reduction loop is the pairwise
 * summation of `pairwise_sum_DOUBLE` (numpy/_core/src/umath/loops_utils.h.src), namely
 *
 *   - n < 8            : a plain left-to-right loop from 0.0;
 *   - 8 <= n <= 128    : eight interleaved accumulators r[0..7] seeded with the first eight
 *                        values and advanced in steps of eight, combined as
 *                        ((r0+r1)+(r2+r3))+((r4+r5)+(r6+r7)), then a left-to-right loop over
 *                        the up-to-seven remaining values;
 *   - n > 128          : split at n/2 rounded DOWN to a multiple of 8 and recurse, left + right.
 *
 * Measured against numpy 2.4.6 on this machine (arrays of length 1..10007, pseudo-random values
 * over several magnitudes): this reproduction is bit-identical on every one, so numpy's SIMD
 * reduction loop keeps the scalar order here.
 */
double pairwise_sum(const double* a, int64_t n);
double pairwise_sum(const std::vector<double>& a);

/**
 * @brief `np.linalg.norm(v)` for a 1-D float64 vector: sqrt of the sequential dot product.
 *
 * The 2D lumped mass is half an edge length per endpoint and the 2D stiffness weight is its
 * reciprocal (constraints.get_mass_matrix / get_stiffness_matrix), both from
 * `np.linalg.norm(coords[a] - coords[b])`. numpy evaluates that as `sqrt(x.dot(x))`, and
 * `x.dot(x)` on this machine goes to Accelerate's ddot. Measured over 200000 pseudo-random
 * length-2 and length-3 difference vectors with irrational coordinates: Accelerate's result is
 * bit-identical to the plain left-to-right `x*x + y*y (+ z*z)` with no fused multiply-add, so
 * that is what this computes -- hence the named temporaries, which stop the compiler from
 * contracting a multiply and an add into an FMA that would round once instead of twice.
 */
double numpy_norm(const double* v, int64_t n);

/**
 * @brief `np.isclose(a, b, rtol=rtol)` for two float64 scalars, with numpy's default atol.
 *
 * It decides the dhat loop's stopping test (`np.isclose(active_dist, sep, rtol=cfg["rtol"])`), so
 * its exact formula matters: numpy's `within_tol` evaluates
 * `absolute(a - b) <= atol + rtol * absolute(b)`, in that association -- the tolerance is built
 * from the SECOND argument alone, which makes the test asymmetric, and the default atol of 1e-8 is
 * an absolute floor that is not negligible here (the separations are of order 1e-3 solver units,
 * so at the default rtol of 1e-2 the floor contributes about one part in a thousand of the band).
 *
 * A NaN on either side compares false, as it does in numpy; the infinite case numpy special-cases
 * cannot reach this, because both loops test `active_dist == inf` before they get here.
 */
bool numpy_isclose(double a, double b, double rtol, double atol = 1e-8);

/**
 * @brief `np.linalg.det(m)` for a 3x3 float64 matrix.
 *
 * `get_mesh_info` sums `abs(np.linalg.det([e1, e2, e3])) / 6` over the cells of a physical group,
 * and that sum divides every AMIPS weight in the polyfem JSON, so a last-bit difference moves the
 * whole materials block. numpy does NOT evaluate the 3x3 determinant by cofactors: it factors the
 * matrix with LAPACK `dgetrf` and then returns `sign * exp(sum log|U_ii|)`
 * (numpy/linalg/umath_linalg.cpp, `det` via `slogdet_single_element` --
 * the log/exp detour is numpy's, not a reformulation here).
 *
 * The factorization reproduced here is the unblocked right-looking one: pick the first row of
 * largest absolute value in the column, swap, scale the column below the pivot by the RECIPROCAL
 * of the pivot, and update the trailing block with a FUSED multiply-add. Measured against numpy
 * 2.4.6 on this machine (Accelerate BLAS; 5000 matrices -- pseudo-random over several magnitudes
 * and tet edge-vector triples from a bent unit grid): the reciprocal-plus-FMA variant reproduces
 * LAPACK's U diagonal and `np.linalg.det` on 5000 of 5000, while plain multiply-subtract manages
 * 2847 and dividing by the pivot 2345. The FMA is therefore part of the contract -- hence
 * std::fma, which must not be relaxed into a multiply and a subtract.
 *
 * @param rows the three matrix ROWS, i.e. rows[i] is the i-th row of `np.stack([e1, e2, e3])`.
 */
double numpy_det3(const double rows[3][3]);

} // namespace wmtk::components::polyfem_ops
