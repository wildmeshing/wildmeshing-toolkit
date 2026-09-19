#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/polyfem_ops/PolyfemRunner.hpp>
#include <wmtk/components/polyfem_ops/polyfem_ops.hpp>

#include <polyfem/State.hpp>
#include <polyfem/mesh/Mesh.hpp>
#include <polyfem/solver/NLProblem.hpp>
#include <polyfem/solver/forms/Form.hpp>
#include <polyfem/solver/forms/lagrangian/AugmentedLagrangianForm.hpp>

#include <mshio/mshio.h>
#include <nlohmann/json.hpp>

#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

using wmtk::components::polyfem_ops::in_process_backend;
using wmtk::components::polyfem_ops::OrderedJson;
using wmtk::components::polyfem_ops::polyfem_ops;
using wmtk::components::polyfem_ops::prepare_operation;
using wmtk::components::polyfem_ops::prepare_state;
using wmtk::components::polyfem_ops::PreparedOperation;
using wmtk::components::polyfem_ops::SolveInputs;
using wmtk::components::polyfem_ops::split_lines;

namespace fs = std::filesystem;

namespace {

// ---------------------------------------------------------------------------
// Meshes
// ---------------------------------------------------------------------------

/// One physical group of a mesh: its name and its cells, as 1-based node tags.
using Group = std::pair<std::string, std::vector<std::vector<size_t>>>;

/// A physical-groups .msh laid out the way pysimwild's conftest writes its fixtures
/// (`_write_groups_msh`): nodes 1..n on the first entity, then one entity and one physical group
/// per group, in order. Binary msh 4.1, so the coordinates are stored exactly.
mshio::MshSpec groups_msh(
    const int dim,
    const std::vector<std::array<double, 3>>& coords,
    const std::vector<Group>& groups)
{
    mshio::MshSpec spec;
    spec.mesh_format.version = "4.1";
    spec.mesh_format.file_type = 1;
    spec.mesh_format.data_size = sizeof(size_t);

    mshio::NodeBlock nodes;
    nodes.entity_dim = dim;
    nodes.entity_tag = 1;
    nodes.num_nodes_in_block = coords.size();
    for (size_t i = 0; i < coords.size(); ++i) {
        nodes.tags.push_back(i + 1);
        nodes.data.insert(nodes.data.end(), coords[i].begin(), coords[i].end());
    }
    spec.nodes.num_entity_blocks = 1;
    spec.nodes.num_nodes = coords.size();
    spec.nodes.min_node_tag = 1;
    spec.nodes.max_node_tag = coords.size();
    spec.nodes.entity_blocks.push_back(std::move(nodes));

    size_t next_id = 1;
    for (size_t g = 0; g < groups.size(); ++g) {
        const int tag = int(g) + 1;
        spec.physical_groups.push_back({dim, tag, groups[g].first});
        if (dim == 3) {
            spec.entities.volumes.push_back({tag, 0, 0, 0, 0, 0, 0, {tag}, {}});
        } else {
            spec.entities.surfaces.push_back({tag, 0, 0, 0, 0, 0, 0, {tag}, {}});
        }
        if (groups[g].second.empty()) {
            continue;
        }
        mshio::ElementBlock block;
        block.entity_dim = dim;
        block.entity_tag = tag;
        block.element_type = dim == 3 ? 4 : 2;
        block.num_elements_in_block = groups[g].second.size();
        for (const auto& cell : groups[g].second) {
            block.data.push_back(next_id++);
            block.data.insert(block.data.end(), cell.begin(), cell.end());
        }
        spec.elements.entity_blocks.push_back(std::move(block));
    }
    spec.elements.num_entity_blocks = spec.elements.entity_blocks.size();
    spec.elements.num_elements = next_id - 1;
    spec.elements.min_element_tag = 1;
    spec.elements.max_element_tag = next_id - 1;
    return spec;
}

/// pysimwild's `_bend`: scale by sqrt(2) and offset by a fraction well below the grid spacing.
/// No coordinate is then exactly representable, so every volume, length and cotangent rounds --
/// which is where a different summation order, or a round trip through text that lost a bit,
/// would show. On an integer grid both would pass unnoticed.
double bend(const double value, const size_t seed)
{
    return value * std::sqrt(2.0) + 0.05 * std::sin(double(seed));
}

/// conftest's `make_two_boxes_3d`, bent: a 7x4x4 grid of cubes, six tetrahedra each; tag_0 is the
/// cube column at x in [1, 2] and tag_1 the one at x in [3, 4] (both y, z in [1, 3]), ambient
/// everywhere else, so the two bodies face each other across one ambient cube.
mshio::MshSpec boxes_3d()
{
    const int nx = 7, ny = 4, nz = 4;
    const auto nid = [&](const int i, const int j, const int k) {
        return size_t(1 + i + j * (nx + 1) + k * (nx + 1) * (ny + 1));
    };
    std::vector<std::array<double, 3>> coords(size_t((nx + 1) * (ny + 1) * (nz + 1)));
    for (int k = 0; k <= nz; ++k) {
        for (int j = 0; j <= ny; ++j) {
            for (int i = 0; i <= nx; ++i) {
                const size_t n = nid(i, j, k) - 1;
                coords[n] = {bend(i, 3 * n), bend(j, 3 * n + 7), bend(k, 3 * n + 14)};
            }
        }
    }

    // One tetrahedron per ordering of the three axes, walking from the cube's low corner to its
    // high one (Freudenthal); the odd orderings get two vertices swapped so that every
    // tetrahedron is positively oriented, which polyfem requires.
    const std::array<std::array<int, 3>, 6> orders{
        {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}}};
    const std::array<bool, 6> odd{false, true, true, false, false, true};
    std::vector<Group> groups{{"ambient", {}}, {"tag_0", {}}, {"tag_1", {}}};
    for (int k = 0; k < nz; ++k) {
        for (int j = 0; j < ny; ++j) {
            for (int i = 0; i < nx; ++i) {
                size_t g = 0;
                if (1 <= j && j < 3 && 1 <= k && k < 3 && (i == 1 || i == 3)) {
                    g = i == 1 ? 1 : 2;
                }
                for (size_t o = 0; o < orders.size(); ++o) {
                    std::array<int, 3> p{i, j, k};
                    std::vector<size_t> tet{nid(p[0], p[1], p[2])};
                    for (const int axis : orders[o]) {
                        ++p[size_t(axis)];
                        tet.push_back(nid(p[0], p[1], p[2]));
                    }
                    if (odd[o]) {
                        std::swap(tet[2], tet[3]);
                    }
                    groups[g].second.push_back(tet);
                }
            }
        }
    }
    return groups_msh(3, coords, groups);
}

/// A bent nx x ny grid of squares, two counter-clockwise triangles each, square (i, j) going to
/// `groups[group_of(i, j)]`.
mshio::MshSpec grid_2d(
    const int nx,
    const int ny,
    const std::function<size_t(int, int)>& group_of,
    std::vector<Group> groups)
{
    const auto nid = [&](const int i, const int j) { return size_t(1 + i + j * (nx + 1)); };
    std::vector<std::array<double, 3>> coords(size_t((nx + 1) * (ny + 1)));
    for (int j = 0; j <= ny; ++j) {
        for (int i = 0; i <= nx; ++i) {
            const size_t n = nid(i, j) - 1;
            coords[n] = {bend(i, 3 * n), bend(j, 3 * n + 1), 0.0};
        }
    }
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const size_t a = nid(i, j), b = nid(i + 1, j), c = nid(i, j + 1), d = nid(i + 1, j + 1);
            auto& cells = groups[group_of(i, j)].second;
            cells.push_back({a, b, d});
            cells.push_back({a, d, c});
        }
    }
    return groups_msh(2, coords, groups);
}

/// The 2D counterpart of `boxes_3d`: tag_0 is the square column at x in [1, 2], tag_1 the one at
/// x in [3, 4] (both y in [1, 3]).
mshio::MshSpec squares_2d()
{
    return grid_2d(
        7,
        4,
        [](const int i, const int j) -> size_t {
            return 1 <= j && j < 3 && (i == 1 || i == 3) ? (i == 1 ? 1 : 2) : 0;
        },
        {{"ambient", {}}, {"tag_0", {}}, {"tag_1", {}}});
}

/// conftest's `make_jagged_2d`, bent: tag_0 is the squares with i < 4 + (j % 2), so the interface
/// zigzags between x = 4 and x = 5.
mshio::MshSpec jagged_2d()
{
    return grid_2d(
        16,
        8,
        [](const int i, const int j) -> size_t { return i < 4 + (j % 2) ? 1 : 0; },
        {{"ambient", {}}, {"tag_0", {}}});
}

/// The two sides every separation case pushes apart, as pysimwild's parity tests spell them:
/// each body's skin against the ambient, two collision bodies with ids 1 and 2.
nlohmann::json both_skins()
{
    return nlohmann::json::array({nlohmann::json::array(
        {{{"region", "tag_0"}, {"filter", "ambient"}},
         {{"region", "tag_1"}, {"filter", "ambient"}}})});
}

std::string read_file(const fs::path& path)
{
    std::ifstream in(path, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

/// Every file in `dir`, by name, with its bytes.
std::map<std::string, std::string> directory_contents(const fs::path& dir)
{
    std::map<std::string, std::string> out;
    for (const auto& entry : fs::directory_iterator(dir)) {
        out[entry.path().filename().string()] = read_file(entry.path());
    }
    return out;
}

/// A fresh directory for one case, and the case's input mesh saved in it.
fs::path case_input(const std::string& name, const mshio::MshSpec& mesh)
{
    const fs::path root = fs::temp_directory_path() / "wmtk_polyfem_ops_in_memory" / name;
    fs::remove_all(root);
    fs::create_directories(root);
    mshio::save_msh((root / "input.msh").string(), mesh);
    return root;
}

// ---------------------------------------------------------------------------
// The two ways of building a State
// ---------------------------------------------------------------------------

/// The simulation JSON of `prepared` as the backend loads it -- `root_path` defaulted to the file,
/// the output directory set as `-o` sets it -- and single-threaded: polyfem's multithreaded
/// assembly sums in scheduling order (measured: ~1e-11 apart between two runs on the same files),
/// and the point here is exact equality.
nlohmann::json solve_args(const PreparedOperation& prepared)
{
    nlohmann::json args;
    std::ifstream(prepared.sim_json_path) >> args;
    args["root_path"] = prepared.sim_json_path.string();
    args["output"]["directory"] = prepared.sim_out_dir.string();
    args["solver"]["max_threads"] = 1;
    return args;
}

/// The route polyfem takes when it reads its inputs from the files the JSON names -- the
/// executable's, and this backend's before it held them in memory.
void state_from_files(polyfem::State& state, const nlohmann::json& args)
{
    state.init(args, /*strict_validation=*/true);
    state.load_mesh(/*non_conforming=*/false, {}, {}, {});
    state.stats.compute_mesh_stats(*state.mesh);
    state.build_basis();
    state.assemble_rhs();
    state.assemble_mass_mat();
}

/// A State up to the forms (what `solve_problem` builds before its first Newton step), or the
/// message of the exception that stopped it.
struct Route
{
    std::unique_ptr<polyfem::State> state = std::make_unique<polyfem::State>();
    Eigen::MatrixXd sol;
    std::optional<std::string> error;
};

Route build_route(const std::function<void(polyfem::State&)>& prepare)
{
    Route route;
    try {
        prepare(*route.state);
        Eigen::MatrixXd pressure;
        route.state->init_solve(route.sol, pressure);
        route.state->init_nonlinear_tensor_solve(route.sol, 1.0, true);
    } catch (const std::exception& e) {
        route.error = e.what();
    }
    return route;
}

// ---------------------------------------------------------------------------
// Exact comparison
// ---------------------------------------------------------------------------

template <typename A, typename B>
bool same(const A& a, const B& b)
{
    return a.rows() == b.rows() && a.cols() == b.cols() && (a.array() == b.array()).all();
}

bool same_sparse(polyfem::StiffnessMatrix a, polyfem::StiffnessMatrix b)
{
    a.makeCompressed();
    b.makeCompressed();
    return a.rows() == b.rows() && a.cols() == b.cols() && a.nonZeros() == b.nonZeros() &&
           std::equal(
               a.outerIndexPtr(),
               a.outerIndexPtr() + a.outerSize() + 1,
               b.outerIndexPtr()) &&
           std::equal(a.innerIndexPtr(), a.innerIndexPtr() + a.nonZeros(), b.innerIndexPtr()) &&
           std::equal(a.valuePtr(), a.valuePtr() + a.nonZeros(), b.valuePtr());
}

Eigen::MatrixXi element_vertices(const polyfem::mesh::Mesh& mesh)
{
    const int nv = mesh.is_volume() ? 4 : 3;
    Eigen::MatrixXi out(mesh.n_elements(), nv);
    for (int e = 0; e < mesh.n_elements(); ++e) {
        for (int i = 0; i < nv; ++i) out(e, i) = mesh.element_vertex(e, i);
    }
    return out;
}

std::vector<int> boundary_ids(const polyfem::mesh::Mesh& mesh)
{
    std::vector<int> out;
    for (int p = 0; p < mesh.n_boundary_elements(); ++p) out.push_back(mesh.get_boundary_id(p));
    return out;
}

/// Everything the two routes build, compared with ==. Returns how many (vertex, vertex) pairs of
/// the collision mesh may collide, and out of how many, so that a caller can see the body ids took
/// effect.
std::pair<long, long> check_states_equal(Route& a, Route& b)
{
    polyfem::State& sa = *a.state;
    polyfem::State& sb = *b.state;

    Eigen::MatrixXd va, vb;
    sa.get_vertices(va);
    sb.get_vertices(vb);
    CHECK(same(va, vb));
    CHECK(same(element_vertices(*sa.mesh), element_vertices(*sb.mesh)));
    // Mesh::orders() is not compared: a .msh file read by polyfem gives all ones on a 3D mesh, and
    // `Mesh::create(V, F)`, which the in-memory route builds the mesh with, gives an empty list.
    // Every element is linear, so both build the same basis, which the comparisons below check.
    CHECK(sa.mesh->is_rational() == sb.mesh->is_rational());
    CHECK(sa.mesh->get_body_ids() == sb.mesh->get_body_ids());
    CHECK(boundary_ids(*sa.mesh) == boundary_ids(*sb.mesh));
    CHECK(sa.n_bases == sb.n_bases);
    CHECK(same(sa.in_node_to_node, sb.in_node_to_node));
    CHECK(sa.boundary_nodes == sb.boundary_nodes);
    CHECK(same_sparse(sa.mass, sb.mass));
    CHECK(same(sa.rhs, sb.rhs));
    CHECK(same(a.sol, b.sol));

    const ipc::CollisionMesh& ca = sa.collision_mesh;
    const ipc::CollisionMesh& cb = sb.collision_mesh;
    CHECK(same(ca.rest_positions(), cb.rest_positions()));
    CHECK(same(ca.edges(), cb.edges()));
    CHECK(same(ca.faces(), cb.faces()));
    CHECK(same(ca.codim_vertices(), cb.codim_vertices()));
    CHECK(same(ca.to_full_vertex_id(), cb.to_full_vertex_id()));
    // The displacement map has no accessor; it is compared through what it does to one
    // displacement of the full mesh with no two entries alike, which tells every row's source
    // node and weight apart.
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> uniform(-1, 1);
    const Eigen::MatrixXd full =
        Eigen::MatrixXd::NullaryExpr(sa.n_bases, sa.mesh->dimension(), [&]() {
            return uniform(rng);
        });
    CHECK(same(ca.map_displacements(full), cb.map_displacements(full)));
    long pairs = 0, allowed = 0, differ = 0;
    for (size_t i = 0; i < ca.num_vertices(); ++i) {
        for (size_t j = 0; j < ca.num_vertices(); ++j) {
            const bool x = ca.can_collide(i, j);
            ++pairs;
            allowed += x;
            differ += x != cb.can_collide(i, j);
        }
    }
    CHECK(differ == 0);

    // Every form, the augmented Lagrangian ones (the hard constraints) included, at the rest point
    // and at a small perturbation of it.
    std::vector<std::shared_ptr<polyfem::solver::Form>> fa = sa.solve_data.nl_problem->forms();
    std::vector<std::shared_ptr<polyfem::solver::Form>> fb = sb.solve_data.nl_problem->forms();
    fa.insert(fa.end(), sa.solve_data.al_form.begin(), sa.solve_data.al_form.end());
    fb.insert(fb.end(), sb.solve_data.al_form.begin(), sb.solve_data.al_form.end());
    REQUIRE(fa.size() == fb.size());
    for (int k = 0; k < 2; ++k) {
        Eigen::VectorXd x = a.sol.col(0);
        if (k == 1) {
            x += 1e-6 * Eigen::VectorXd::NullaryExpr(x.size(), [&]() { return uniform(rng); });
        }
        for (size_t i = 0; i < fa.size(); ++i) {
            const polyfem::solver::Form& form = *fa[i];
            INFO("form " << i << " (" << typeid(form).name() << "), point " << k);
            fa[i]->solution_changed(x);
            fb[i]->solution_changed(x);
            CHECK(fa[i]->value(x) == fb[i]->value(x));
            Eigen::VectorXd ga, gb;
            fa[i]->first_derivative(x, ga);
            fb[i]->first_derivative(x, gb);
            CHECK(same(ga, gb));
            polyfem::StiffnessMatrix ha, hb;
            fa[i]->second_derivative(x, ha);
            fb[i]->second_derivative(x, hb);
            CHECK(same_sparse(ha, hb));
        }
    }
    return {allowed, pairs};
}

/// Run one operation on `mesh` twice -- inputs_only, which writes every input, and a normal run's
/// preparation, which writes none but the JSON and the OBJ -- build a State from each (polyfem's
/// file route, and `prepare_state` on the in-memory content), and require them to be the same.
/// `edit` is applied to both simulation JSONs alike before either State is built.
///
/// @return the collision filter's (allowed, total) vertex pairs, or nothing when both routes
/// threw, which they must do with the same message.
std::optional<std::pair<long, long>> check_routes_agree(
    const std::string& name,
    const mshio::MshSpec& mesh,
    nlohmann::json params,
    const std::function<void(nlohmann::json&)>& edit = [](nlohmann::json&) {})
{
    const fs::path root = case_input(name, mesh);
    params["application"] = "polyfem_ops";
    params["input"] = (root / "input.msh").string();

    params["output"] = (root / "files" / "out").string();
    params["inputs_only"] = true;
    const PreparedOperation files = prepare_operation(params);

    params["output"] = (root / "memory" / "out").string();
    params["inputs_only"] = false;
    const PreparedOperation memory = prepare_operation(params);

    // Nothing the in-memory route could read is on disk: its input directory holds the simulation
    // JSON and the OBJ, and polyfem reads neither of those from there.
    std::set<std::string> on_disk;
    for (const auto& [file, bytes] : directory_contents(memory.sim_json_path.parent_path())) {
        on_disk.insert(file);
    }
    REQUIRE(
        on_disk ==
        std::set<std::string>{memory.sim_json_path.filename().string(), "interface_collision.obj"});

    nlohmann::json files_args = solve_args(files);
    nlohmann::json memory_args = solve_args(memory);
    edit(files_args);
    edit(memory_args);
    Route from_files = build_route([&](polyfem::State& s) { state_from_files(s, files_args); });
    Route from_memory = build_route(
        [&](polyfem::State& s) { prepare_state(s, memory_args, memory.inputs, nullptr); });

    REQUIRE(from_files.error == from_memory.error);
    if (from_files.error.has_value()) {
        return std::nullopt;
    }
    return check_states_equal(from_files, from_memory);
}

// ---------------------------------------------------------------------------
// The two-cube scene of the active-distance test
// ---------------------------------------------------------------------------

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

/// Two cubes with a `gap` between them along x, as one mesh with one group.
mshio::MshSpec two_cubes(const double gap)
{
    std::vector<std::array<double, 3>> coords;
    std::vector<std::vector<size_t>> cells;
    for (const double x0 : {0.0, 1.0 + gap}) {
        Eigen::MatrixXd V;
        Eigen::MatrixXi T;
        cube(x0, V, T);
        const size_t offset = coords.size();
        for (int v = 0; v < V.rows(); ++v) coords.push_back({V(v, 0), V(v, 1), V(v, 2)});
        for (int t = 0; t < T.rows(); ++t) {
            std::vector<size_t> cell;
            for (int k = 0; k < 4; ++k) cell.push_back(offset + size_t(T(t, k)) + 1);
            cells.push_back(cell);
        }
    }
    return groups_msh(3, coords, {{"body", cells}});
}

/// A simulation JSON of the shape the operations build: AMIPS, the smooth contact formulation,
/// one quasistatic step and every boundary node fixed. No collision proxy and no constraints, so
/// the whole scene is the one mesh -- everything else this port generates is orthogonal to what is
/// measured here.
nlohmann::json two_cube_json(
    const std::string& msh,
    const fs::path& out_dir,
    const double scale,
    const double dhat)
{
    nlohmann::json geometry;
    geometry["mesh"] = msh;
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

/// The active distance on the LAST line of polyfem's output that carries one -- it logs
/// "Minimum distance during solve: <d>, active distance: <a>, dhat: <h>" after every Newton step,
/// and the converged state is the last -- read with `strtod`, which stops at the comma. polyfem
/// prints the value with enough digits to round trip, so this recovers the double exactly.
std::optional<double> logged_active_distance(const std::vector<std::string>& lines)
{
    static const std::string marker = "active distance:";
    for (auto it = lines.rbegin(); it != lines.rend(); ++it) {
        const size_t at = it->rfind(marker);
        if (at != std::string::npos) {
            return std::strtod(it->c_str() + at + marker.size(), nullptr);
        }
    }
    return std::nullopt;
}

} // namespace

// The in-process backend reports the active distance off the contact form instead of off the log
// text. The two must be the same number, not a number that rounds to the same print: polyfem logs
// it with enough digits to round trip, so the check is for equality.
TEST_CASE("polyfem_ops in-process active distance is the logged one", "[components][polyfem_ops]")
{
    const fs::path root = fs::temp_directory_path() / "wmtk_polyfem_ops_in_process";
    fs::remove_all(root);
    const fs::path out_dir = root / "out";
    fs::create_directories(out_dir);

    // Gap 0.1 in mesh units, scale 1, dhat 0.2: the two cubes are inside the barrier's support at
    // rest, so the contact form has a non-empty collision set from the first evaluation and every
    // Newton step logs the line. The mesh the JSON names is never written: it is in memory only.
    const std::string msh = (root / "two_cubes.msh").string();
    const fs::path json_path = root / "two_cubes.json";
    {
        std::ofstream(json_path) << two_cube_json(msh, out_dir, 1.0, 0.2).dump(4);
    }
    SolveInputs inputs;
    inputs.meshes.emplace(msh, two_cubes(0.1));

    const fs::path log_path = out_dir / "polyfem.log";
    auto backend = in_process_backend(std::move(inputs));
    const auto result = backend->solve(json_path, out_dir, log_path);

    REQUIRE(result.returncode == 0);
    REQUIRE(result.active_distance.has_value());

    // What the Python engine reads out of the executable's output for the same solve.
    const std::optional<double> logged = logged_active_distance(result.lines);
    REQUIRE(logged.has_value());
    CHECK(*result.active_distance == *logged);

    // ... and the log file on disk carries that same output, which is what makes the file still
    // worth keeping: it is the same text, not a summary of it.
    const std::optional<double> from_file =
        logged_active_distance(split_lines(read_file(log_path)));
    REQUIRE(from_file.has_value());
    CHECK(*from_file == *logged);
}

// A normal run hands polyfem its inputs in memory; inputs_only writes them to the files the
// simulation JSON names, and polyfem can read those itself. Both routes must build the SAME State,
// to the last bit: the mesh (vertices, elements, body ids), the basis and its boundary, the mass
// matrix and right-hand side, the collision mesh with its displacement map and body-id filter,
// and every energy term's value, gradient and Hessian. Single-threaded, see `solve_args`.
//
// The body ids are what this used to fail on: polyfem's in-memory `load_mesh(V, F)` never sets
// them, so every material would silently have covered every element. `prepare_state` sets them
// itself and then applies the geometry entry's transformation exactly as the file route does.
TEST_CASE("polyfem_ops in-memory inputs build the file route's State", "[components][polyfem_ops]")
{
    SECTION("3D separation: pins without and with axes, a collision proxy with body ids")
    {
        const auto filter = check_routes_agree(
            "sep3d",
            boxes_3d(),
            {{"operation", "minimum_separation"},
             {"collision_pairs", both_skins()},
             {"sep", 1.5e-3},
             {"protected_regions", {"tag_0", {{"region", "tag_1"}, {"axes", "xy"}}}}});
        REQUIRE(filter.has_value());
        // Two collision bodies: the body ids reached the collision mesh and forbid some pairs.
        CHECK(filter->first < filter->second);
    }
    SECTION("2D separation")
    {
        const auto filter = check_routes_agree(
            "sep2d",
            squares_2d(),
            {{"operation", "minimum_separation"},
             {"collision_pairs", both_skins()},
             {"sep", 1.5e-3}});
        REQUIRE(filter.has_value());
        CHECK(filter->first < filter->second);
    }
    SECTION("2D laplacian smoothing")
    {
        // Positions mode (the default), so the Laplacian's right-hand side is not zero.
        CHECK(check_routes_agree(
                  "smooth2d",
                  jagged_2d(),
                  {{"operation", "laplacian_smoothing"},
                   {"interfaces", {{{"region", "tag_0"}, {"filter", "ambient"}}}}})
                  .has_value());
    }
    SECTION("3D laplacian smoothing")
    {
        CHECK(check_routes_agree(
                  "smooth3d",
                  boxes_3d(),
                  {{"operation", "laplacian_smoothing"},
                   {"interfaces",
                    {{{"region", "tag_0"}, {"filter", "ambient"}},
                     {{"region", "tag_1"}, {"filter", "ambient"}}}}})
                  .has_value());
    }
    // The spec takes any float for `scale`, and a negative one reaches the geometry block, the
    // collision proxy's transformation and the AMIPS volume normalization alike.
    SECTION("2D separation, negative scale")
    {
        CHECK(check_routes_agree(
                  "sep2d_negative",
                  squares_2d(),
                  {{"operation", "minimum_separation"},
                   {"collision_pairs", both_skins()},
                   {"sep", 1.5e-3},
                   {"scale", -1e-3}})
                  .has_value());
    }
    SECTION("3D separation, negative scale")
    {
        // A negative scale mirrors a 3D mesh, which inverts every tetrahedron. Measured: both
        // routes refuse it in build_basis with "element 0 is flipped, type Simplex".
        CHECK_FALSE(check_routes_agree(
                        "sep3d_negative",
                        boxes_3d(),
                        {{"operation", "minimum_separation"},
                         {"collision_pairs", both_skins()},
                         {"sep", 1.5e-3},
                         {"scale", -1e-3},
                         {"protected_regions", {"tag_0"}}})
                        .has_value());
    }
    // In 2D a negative scalar scale is a half turn, not a mirror, so none of the cases above can
    // tell whether `prepare_state` copies the mesh the way `read_fem_geometry` does. A per-axis
    // scale can, and polyfem's geometry block takes one, so both documents are edited to mirror x.
    // Measured with the copy removed from `prepare_state`: the in-memory route alone throws
    // "element 0 is flipped".
    SECTION("2D separation, a JSON that mirrors x")
    {
        CHECK(check_routes_agree(
                  "sep2d_mirror",
                  squares_2d(),
                  {{"operation", "minimum_separation"},
                   {"collision_pairs", both_skins()},
                   {"sep", 1.5e-3}},
                  [](nlohmann::json& args) {
                      args["geometry"][0]["transformation"]["scale"] = {-1e-3, 1e-3};
                  })
                  .has_value());
    }
}

// A normal run writes no input file but the OBJ, which is output only (it shows which faces the
// selection picked), and writes the same simulation JSON as inputs_only -- the reduced mesh's
// volumes, which divide every AMIPS weight, now come from memory instead of from the .msh read
// back, in the same order. The meshes are bent so that order could show.
TEST_CASE("polyfem_ops normal run writes only the JSON and the OBJ", "[components][polyfem_ops]")
{
    // Both runs go to the same output stem, so the paths inside the two JSONs are the same too.
    const auto run_both = [](const std::string& name,
                             const mshio::MshSpec& mesh,
                             nlohmann::json params,
                             const std::string& sim_in) {
        const fs::path root = case_input(name, mesh);
        params["application"] = "polyfem_ops";
        params["input"] = (root / "input.msh").string();
        params["output"] = (root / "out").string();
        params["inputs_only"] = true;
        polyfem_ops(params);
        const auto inputs_only = directory_contents(root / sim_in);
        fs::remove_all(root / sim_in);

        params["inputs_only"] = false;
        polyfem_ops(params);
        REQUIRE(fs::is_regular_file(root / "out.msh")); // the solve ran and wrote its result
        return std::make_pair(inputs_only, directory_contents(root / sim_in));
    };

    SECTION("2D smoothing: the JSON byte for byte")
    {
        const auto [inputs_only, normal] = run_both(
            "normal_smooth2d",
            jagged_2d(),
            {{"operation", "laplacian_smoothing"},
             {"interfaces", {{{"region", "tag_0"}, {"filter", "ambient"}}}}},
            "smooth_input");
        CHECK(inputs_only.size() == 5); // the JSON, the OBJ, two constraints, the reduced mesh
        REQUIRE(normal.size() == 2);
        CHECK(normal.at("interface_collision.obj") == inputs_only.at("interface_collision.obj"));
        // The single solve writes the document again unchanged, so the file on disk is still the
        // generated one.
        CHECK(normal.at("smoothing.json") == inputs_only.at("smoothing.json"));
    }
    SECTION("3D separation with pins: the JSON byte for byte, but for the loop's own keys")
    {
        // sep 5e-4 against a gap of about 1.4e-3 at scale 1e-3: the probe finds the bodies already
        // separated, so one solve runs and the loop stops.
        const auto [inputs_only, normal] = run_both(
            "normal_sep3d",
            boxes_3d(),
            {{"operation", "minimum_separation"},
             {"collision_pairs", both_skins()},
             {"sep", 5e-4},
             {"protected_regions", {"tag_0", {{"region", "tag_1"}, {"axes", "z"}}}}},
            "sep_input");
        // The JSON, the OBJ, two constraints, the linear map, the body ids, the reduced mesh and
        // the two pin files.
        CHECK(inputs_only.size() == 9);
        REQUIRE(normal.size() == 2);
        CHECK(normal.at("interface_collision.obj") == inputs_only.at("interface_collision.obj"));

        // The dhat loop rewrites the document before every solve: it sets contact.dhat and the
        // barrier stiffness, and adds the two warm-start paths. Those four keys are the loop's;
        // put back to the generated values, the rest has to be the generated document exactly.
        const std::string& generated_text = inputs_only.at("separation.json");
        const OrderedJson generated = OrderedJson::parse(generated_text);
        OrderedJson last = OrderedJson::parse(normal.at("separation.json"));
        CHECK(last != generated); // the loop did rewrite it
        last["contact"]["dhat"] = generated["contact"]["dhat"];
        last["solver"]["contact"]["barrier_stiffness"] =
            generated["solver"]["contact"]["barrier_stiffness"];
        last["output"]["data"].erase("state");
        last["input"]["data"].erase("state");
        REQUIRE(generated.dump(4) == generated_text); // the parse round trip itself is exact
        CHECK(last.dump(4) == generated_text);
    }
}
