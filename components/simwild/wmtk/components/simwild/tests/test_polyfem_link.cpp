#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <polyfem/State.hpp>
#include <polyfem/solver/forms/ElasticForm.hpp>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

// Smoke test of the polyfem link. A State is built from an in-memory mesh, no files, and the
// AMIPS elastic form is evaluated at rest. With `use_rest_pose` the AMIPS density at F = I is
// trace(I) / det(I)^(2/3) = 3 per unit volume, times the material weight, so the total is
// 3 * weight * volume.
//
// The mesh is the unit cube split into 12 tetrahedra around its centre: each of the 6 faces is
// split into 2 triangles and joined to the centre vertex. polyfem refuses a static problem with
// no Dirichlet nodes, so the whole boundary (the 8 corners) is fixed at zero displacement, as the
// pysimwild operations do; the centre vertex stays free. Volume 1, weight 2: the energy is 6.
// This exercises init, the in-memory mesh loader, basis construction, boundary conditions, the
// assembler and the form layer, nothing else.
TEST_CASE("polyfem link: AMIPS energy at rest on a star-split cube", "[polyfem_ops]")
{
    Eigen::MatrixXd V(9, 3);
    V << 0, 0, 0, //
        1, 0, 0, //
        1, 1, 0, //
        0, 1, 0, //
        0, 0, 1, //
        1, 0, 1, //
        1, 1, 1, //
        0, 1, 1, //
        0.5, 0.5, 0.5;
    // Each row: a face triangle wound so its normal points at the centre, then the centre, which
    // makes every tetrahedron positively oriented (checked below; AMIPS is NaN on an inverted one).
    Eigen::MatrixXi T(12, 4);
    T << 0, 1, 2, 8, //  bottom (z = 0)
        0, 2, 3, 8, //
        4, 6, 5, 8, //  top (z = 1)
        4, 7, 6, 8, //
        0, 5, 1, 8, //  front (y = 0)
        0, 4, 5, 8, //
        3, 6, 7, 8, //  back (y = 1)
        3, 2, 6, 8, //
        0, 7, 4, 8, //  left (x = 0)
        0, 3, 7, 8, //
        1, 6, 2, 8, //  right (x = 1)
        1, 5, 6, 8;
    double volume = 0;
    for (int t = 0; t < T.rows(); ++t) {
        Eigen::Matrix3d edges;
        for (int k = 0; k < 3; ++k) edges.col(k) = V.row(T(t, k + 1)) - V.row(T(t, 0));
        const double v = edges.determinant() / 6.0;
        REQUIRE(v > 0);
        volume += v;
    }
    REQUIRE_THAT(volume, Catch::Matchers::WithinRel(1.0, 1e-12));

    // "geometry" is required by polyfem's input spec; with load_mesh(V, T) the path is never opened.
    nlohmann::json args = {
        {"geometry", nlohmann::json::array({{{"mesh", "in-memory"}}})},
        {"materials",
         nlohmann::json::array({{{"type", "AMIPS"}, {"use_rest_pose", true}, {"weight", 2.0}}})},
        {"boundary_conditions",
         {{"dirichlet_boundary",
           nlohmann::json::array(
               {{{"id", "all"}, {"value", {0.0, 0.0, 0.0}}, {"dimension", {true, true, true}}}})}}},
    };

    polyfem::State state;
    state.init(args, /*strict_validation=*/true);
    state.load_mesh(V, T);
    state.build_basis();
    state.assemble_rhs();
    state.assemble_mass_mat();

    // init_solve sizes the solution (zeros here: no initial condition) and builds the rhs
    // assembler, exactly as State::solve_problem does before the nonlinear setup.
    Eigen::MatrixXd sol, pressure;
    state.init_solve(sol, pressure);
    state.init_nonlinear_tensor_solve(sol);
    REQUIRE(state.solve_data.elastic_form != nullptr);

    const double energy = state.solve_data.elastic_form->value(sol);
    CHECK_THAT(energy, Catch::Matchers::WithinRel(6.0, 1e-10));
}
