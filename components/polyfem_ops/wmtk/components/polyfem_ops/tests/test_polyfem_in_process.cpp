#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/polyfem_ops/PolyfemRunner.hpp>

#include <polyfem/State.hpp>
#include <polyfem/io/MshWriter.hpp>
#include <polyfem/mesh/Mesh.hpp>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>

using wmtk::components::polyfem_ops::in_process_backend;
using wmtk::components::polyfem_ops::parse_active_distance;
using wmtk::components::polyfem_ops::split_lines;

namespace {

/// A unit cube star-split into 12 tetrahedra around its centre, translated along x. The same
/// construction the polyfem link test uses; two of them make a contact scene with a flat gap.
void cube(const double x0, Eigen::MatrixXd& V, Eigen::MatrixXi& T)
{
    V.resize(9, 3);
    V << 0, 0, 0, //
        1, 0, 0, //
        1, 1, 0, //
        0, 1, 0, //
        0, 0, 1, //
        1, 0, 1, //
        1, 1, 1, //
        0, 1, 1, //
        0.5, 0.5, 0.5;
    V.col(0).array() += x0;
    // Each row: a face triangle wound so its normal points at the centre, then the centre.
    T.resize(12, 4);
    T << 0, 1, 2, 8, //
        0, 2, 3, 8, //
        4, 6, 5, 8, //
        4, 7, 6, 8, //
        0, 5, 1, 8, //
        0, 4, 5, 8, //
        3, 6, 7, 8, //
        3, 2, 6, 8, //
        0, 7, 4, 8, //
        0, 3, 7, 8, //
        1, 6, 2, 8, //
        1, 5, 6, 8;
}

/// Two cubes with a `gap` between them along x, as one mesh. Written as an ASCII .msh, because
/// the point of these two tests is what polyfem's geometry reader does with a FILE.
std::filesystem::path write_two_cubes(const std::filesystem::path& path, const double gap)
{
    Eigen::MatrixXd V0, V1;
    Eigen::MatrixXi T0, T1;
    cube(0.0, V0, T0);
    cube(1.0 + gap, V1, T1);

    Eigen::MatrixXd V(V0.rows() + V1.rows(), 3);
    V << V0, V1;
    Eigen::MatrixXi T(T0.rows() + T1.rows(), 4);
    T << T0, T1.array() + int(V0.rows());
    const std::vector<int> body_ids(T.rows(), 1);

    std::filesystem::create_directories(path.parent_path());
    polyfem::io::MshWriter::write(path.string(), V, T, body_ids, /*is_volume=*/true);
    return path;
}

/// A simulation JSON of the shape the operations build: AMIPS, the smooth contact formulation,
/// one quasistatic step and every boundary node fixed. No collision proxy and no constraint
/// files, so the whole scene is the one .msh -- everything else this port writes is orthogonal to
/// what is measured here.
nlohmann::json two_cube_json(
    const std::filesystem::path& msh,
    const std::filesystem::path& out_dir,
    const double scale,
    const double dhat)
{
    nlohmann::json geometry;
    geometry["mesh"] = msh.string();
    geometry["transformation"]["scale"] = scale;

    nlohmann::json doc;
    doc["geometry"] = nlohmann::json::array({geometry});
    doc["materials"] = {{"type", "AMIPS"}, {"weight", 1.0}, {"use_rest_pose", true}};
    doc["contact"] = {
        {"enabled", true},
        {"friction_coefficient", 0.0},
        {"use_gcp_formulation", true},
        {"dhat", dhat},
        {"alpha_t", 0.1},
        {"alpha_n", 0.1},
        // polyfem's spec defaults this to true and the ipc-toolkit this links against has no such
        // field; the operations' own JSON sets it false for the same reason.
        {"use_rest_shape_measure", false}};
    doc["solver"]["nonlinear"]["max_iterations"] = 10;
    doc["solver"]["nonlinear"]["allow_out_of_iterations"] = true;
    doc["solver"]["contact"]["barrier_stiffness"] = 1e2;
    doc["time"] = {{"quasistatic", true}, {"dt", 1}, {"time_steps", 1}};
    doc["boundary_conditions"]["rhs"] = std::vector<double>(3, 0.0);
    doc["boundary_conditions"]["dirichlet_boundary"] = nlohmann::json::array(
        {{{"id", "all"},
          {"value", {0.0, 0.0, 0.0}},
          {"dimension", {true, true, true}}}});
    doc["output"]["data"]["solution"] = (out_dir / "solution.txt").string();
    doc["output"]["data"]["advanced"]["reorder_nodes"] = true;
    doc["output"]["paraview"]["file_name"] = "";
    return doc;
}

std::string read_file(const std::filesystem::path& path)
{
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

} // namespace

// The in-process backend reports the active distance off the contact form instead of off the log
// text. The two must be the same number, not a number that rounds to the same print: polyfem logs
// it with enough digits to round trip, so the check is for equality.
TEST_CASE("polyfem_ops in-process active distance is the logged one", "[components][polyfem_ops]")
{
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "wmtk_polyfem_ops_in_process";
    std::filesystem::remove_all(root);
    const std::filesystem::path out_dir = root / "out";
    std::filesystem::create_directories(out_dir);

    // Gap 0.1 in mesh units, scale 1, dhat 0.2: the two cubes are inside the barrier's support at
    // rest, so the contact form has a non-empty collision set from the first evaluation and every
    // Newton step logs the line.
    const std::filesystem::path msh = write_two_cubes(root / "two_cubes.msh", 0.1);
    const std::filesystem::path json_path = root / "two_cubes.json";
    {
        std::ofstream(json_path) << two_cube_json(msh, out_dir, 1.0, 0.2).dump(4);
    }

    const std::filesystem::path log_path = out_dir / "polyfem.log";
    auto backend = in_process_backend();
    const auto result = backend->solve(json_path, out_dir, log_path);

    REQUIRE(result.returncode == 0);
    REQUIRE(result.active_distance.has_value());

    // What the subprocess backend would have read out of the same solve's output.
    const std::optional<double> logged = parse_active_distance(result.lines);
    REQUIRE(logged.has_value());
    CHECK(*result.active_distance == *logged);

    // ... and the log file on disk carries that same output, which is what makes the file still
    // worth keeping: it is the same text, not a summary of it.
    const std::optional<double> from_file = parse_active_distance(split_lines(read_file(log_path)));
    REQUIRE(from_file.has_value());
    CHECK(*from_file == *logged);
}

// Why the reduced mesh keeps being written to a file rather than handed to `load_mesh(V, F)`.
// Two measurements. The geometry block's `scale` IS exactly reproducible: it is one product per
// coordinate, so passing the mesh in memory would lose nothing there. The body ids are not
// reproducible at all -- `load_mesh(V, F)` goes through `Mesh::create(V, F)`, which never calls
// set_body_ids, while `Mesh::create(path)` fills them from the .msh physical groups -- and the
// simulation JSON this port builds declares one material per body id (the reduced mesh's ids are
// 1 for ambient and 2 for the bodies), so losing them would silently give every element the same
// material. That is what keeps the file.
TEST_CASE("polyfem_ops in-memory mesh loses the body ids", "[components][polyfem_ops]")
{
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "wmtk_polyfem_ops_mesh_in_memory";
    std::filesystem::remove_all(root);
    const std::filesystem::path out_dir = root / "out";
    std::filesystem::create_directories(out_dir);
    const std::filesystem::path msh = write_two_cubes(root / "two_cubes.msh", 0.1);

    const double scale = 1e-3; // the value the operations run at
    polyfem::State state;
    state.init(two_cube_json(msh, out_dir, scale, 0.2), /*strict_validation=*/true);
    state.load_mesh(/*non_conforming=*/false, {}, {}, {});
    REQUIRE(state.mesh != nullptr);

    Eigen::MatrixXd transformed;
    state.get_vertices(transformed);

    // The same file, read without the geometry block: `Mesh::create(path)` is what
    // `read_fem_geometry` calls before it applies the transformation.
    const std::unique_ptr<polyfem::mesh::Mesh> raw =
        polyfem::mesh::Mesh::create(msh.string(), /*non_conforming=*/false);
    REQUIRE(raw != nullptr);
    Eigen::MatrixXd rest(raw->n_vertices(), raw->dimension());
    for (int v = 0; v < raw->n_vertices(); ++v) {
        rest.row(v) = raw->point(v);
    }

    // The transformation is A*p + b with A = diag(scale) and b = 0, so every coordinate comes out
    // as the product of two doubles and nothing else: reproducing it needs no geometry reader.
    CHECK(transformed == (rest * scale));

    // The body ids, though, are read by `Mesh::create(path)` from the .msh itself, and the
    // in-memory overload has no path to them at all.
    Eigen::MatrixXi cells(raw->n_cells(), 4);
    for (int c = 0; c < raw->n_cells(); ++c) {
        for (int lv = 0; lv < 4; ++lv) {
            cells(c, lv) = raw->cell_vertex(c, lv);
        }
    }
    const std::unique_ptr<polyfem::mesh::Mesh> in_memory =
        polyfem::mesh::Mesh::create(rest * scale, cells, /*non_conforming=*/false);
    REQUIRE(in_memory != nullptr);
    CHECK(raw->has_body_ids());
    CHECK_FALSE(in_memory->has_body_ids());
}
