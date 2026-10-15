// The hybrid output (filter "hybrid"): tets inside the volume-role inputs, the surface-role sheets
// as a triangle block, the feature curves as a line block and the anchors as 0D points, all in
// <output>_hybrid.msh. These tests run tetwild end to end on a scene with one of each and check
// every block of the written file against the input geometry.
//
// Scene: the cube [0,2]^3 (volume role), the square [0,2]^2 at z = 3 (surface role, an open sheet
// above the cube), the segment (0.5,0.5,2.5)-(1.5,1.5,2.5) between them (input_edges) and the
// point (1,1,2.6) (input_points). The input bounding box is [0,2]x[0,2]x[0,3], so with the default
// eps_rel 1e-3 the envelope is eps = 1e-3 * sqrt(17).

#include <catch2/catch_test_macros.hpp>

#include <igl/per_face_normals.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/unique_edge_map.h>
#include <mshio/mshio.h>
#include <wmtk/components/tetwild/tetwild.hpp>
#include <wmtk/utils/WindingNumberHierarchy.hpp>
#include <wmtk/utils/io.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <vector>

using namespace wmtk;
namespace fs = std::filesystem;

namespace {

const double kEps = 1e-3 * std::sqrt(17.0);
const Vector3d kCurveA(0.5, 0.5, 2.5), kCurveB(1.5, 1.5, 2.5);
const Vector3d kPoint(1.0, 1.0, 2.6);

fs::path scratch_dir()
{
    const fs::path d = fs::temp_directory_path() / "wmtk_test_hybrid_output";
    fs::create_directories(d);
    return d;
}

void write_scene(const fs::path& d)
{
    std::ofstream cube(d / "cube.obj");
    cube << "v 0 0 0\nv 2 0 0\nv 2 2 0\nv 0 2 0\nv 0 0 2\nv 2 0 2\nv 2 2 2\nv 0 2 2\n"
            "f 1 3 2\nf 1 4 3\nf 5 6 7\nf 5 7 8\nf 1 2 6\nf 1 6 5\n"
            "f 3 4 8\nf 3 8 7\nf 2 3 7\nf 2 7 6\nf 1 5 8\nf 1 8 4\n";
    std::ofstream sheet(d / "sheet.obj");
    sheet << "v 0 0 3\nv 2 0 3\nv 2 2 3\nv 0 2 3\nf 1 2 3\nf 1 3 4\n";
    std::ofstream curve(d / "curve.obj");
    curve << "v 0.5 0.5 2.5\nv 1.5 1.5 2.5\nl 1 2\n";
    std::ofstream point(d / "point.obj");
    point << "v 1 1 2.6\n";
}

nlohmann::json scene_config(const fs::path& d, const std::string& name)
{
    return {
        {"application", "tetwild"},
        {"input", {(d / "cube.obj").string(), (d / "sheet.obj").string()}},
        {"input_roles", {"volume", "surface"}},
        {"input_edges", {(d / "curve.obj").string()}},
        {"input_points", {(d / "point.obj").string()}},
        {"filter", "hybrid"},
        {"num_threads", 0},
        {"stop_energy", 10},
        {"max_iterations", 20},
        {"output", (d / name).string()}};
}

double point_segment_distance(const Vector3d& p, const Vector3d& a, const Vector3d& b)
{
    const Vector3d ab = b - a;
    const double t = std::clamp((p - a).dot(ab) / ab.squaredNorm(), 0.0, 1.0);
    return (p - (a + t * ab)).norm();
}

} // namespace

TEST_CASE("hybrid-output-blocks", "[tetwild][hybrid]")
{
    const fs::path d = scratch_dir();
    write_scene(d);
    REQUIRE_NOTHROW(components::tetwild::tetwild(scene_config(d, "scene")));
    const fs::path file = d / "scene_hybrid.msh";
    REQUIRE(fs::exists(file));

    MshData msh;
    msh.load(file.string());

    // a. Tets: some, and each one's centre inside the cube (the volume-role input), up to eps.
    {
        std::vector<Vector3d> V(msh.get_num_tet_vertices());
        msh.extract_tet_vertices([&](size_t i, double x, double y, double z) { V[i] << x, y, z; });
        std::vector<std::array<size_t, 4>> T(msh.get_num_tets());
        msh.extract_tets(
            [&](size_t i, size_t a, size_t b, size_t c, size_t e) { T[i] = {{a, b, c, e}}; });
        REQUIRE(!T.empty());
        size_t outside = 0;
        for (const auto& t : T) {
            const Vector3d g = (V[t[0]] + V[t[1]] + V[t[2]] + V[t[3]]) / 4;
            for (int k = 0; k < 3; ++k) {
                if (g[k] < -kEps || g[k] > 2 + kEps) {
                    ++outside;
                    break;
                }
            }
        }
        CHECK(outside == 0);
    }

    // b. Triangles: the sheet, and only the sheet -- every vertex within eps of the plane z = 3
    // and of the square, none near the cube, and the total area that of the square.
    {
        std::vector<Vector3d> V(msh.get_num_face_vertices());
        msh.extract_face_vertices([&](size_t i, double x, double y, double z) { V[i] << x, y, z; });
        std::vector<std::array<size_t, 3>> F(msh.get_num_faces());
        msh.extract_faces([&](size_t i, size_t a, size_t b, size_t c) { F[i] = {{a, b, c}}; });
        REQUIRE(!F.empty());
        for (const Vector3d& p : V) {
            CHECK(std::abs(p.z() - 3) <= kEps);
            CHECK(p.x() >= -kEps);
            CHECK(p.x() <= 2 + kEps);
            CHECK(p.y() >= -kEps);
            CHECK(p.y() <= 2 + kEps);
        }
        double area = 0;
        for (const auto& f : F) {
            area += 0.5 * (V[f[1]] - V[f[0]]).cross(V[f[2]] - V[f[0]]).norm();
        }
        // Each side of the square may move by at most eps within the envelope.
        CHECK(std::abs(area - 4) <= 4 * 2 * kEps);
    }

    // c. Lines: they tile the input segment -- every vertex inside the feature tube (half of eps
    // by default) around it, and the total length that of the segment.
    {
        std::vector<Vector3d> V(msh.get_num_edge_vertices());
        msh.extract_edge_vertices([&](size_t i, double x, double y, double z) { V[i] << x, y, z; });
        std::vector<std::array<size_t, 2>> E(msh.get_num_edges());
        msh.extract_edges([&](size_t i, size_t a, size_t b) { E[i] = {{a, b}}; });
        REQUIRE(!E.empty());
        for (const Vector3d& p : V) {
            CHECK(point_segment_distance(p, kCurveA, kCurveB) <= 0.5 * kEps);
        }
        double length = 0;
        for (const auto& e : E) length += (V[e[1]] - V[e[0]]).norm();
        CHECK(std::abs(length - (kCurveB - kCurveA).norm()) <= 2 * kEps);
    }

    // d. Points: the anchor is a 0D element (gmsh type 15) within its ball, of radius the feature
    // tube's, around the input point.
    {
        const mshio::MshSpec spec = mshio::load_msh(file.string());
        std::vector<Vector3d> points;
        for (const auto& eb : spec.elements.entity_blocks) {
            if (eb.element_type != 15) continue;
            for (const auto& nb : spec.nodes.entity_blocks) {
                if (nb.entity_dim != 0 || nb.entity_tag != eb.entity_tag) continue;
                for (size_t i = 0; i < nb.num_nodes_in_block; ++i) {
                    points.emplace_back(nb.data[3 * i], nb.data[3 * i + 1], nb.data[3 * i + 2]);
                }
            }
        }
        REQUIRE(!points.empty());
        double best = std::numeric_limits<double>::infinity();
        for (const Vector3d& p : points) best = std::min(best, (p - kPoint).norm());
        CHECK(best <= 0.5 * kEps);
    }
}

TEST_CASE("hybrid-output-role-errors", "[tetwild][hybrid]")
{
    const fs::path d = scratch_dir();
    write_scene(d);

    // One role per input file: two inputs, one role.
    {
        nlohmann::json j = scene_config(d, "wrong_count");
        j["input_roles"] = {"volume"};
        CHECK_THROWS(components::tetwild::tetwild(j));
    }
    // Only "volume" and "surface" are roles.
    {
        nlohmann::json j = scene_config(d, "unknown_role");
        j["input_roles"] = {"volume", "shell"};
        CHECK_THROWS(components::tetwild::tetwild(j));
    }
}

// ---- Scenes on challenging Thingi10K models (hidden: minutes each) ----------------------------
//
// The same checks on real geometry, with every bound taken from the run's own tolerances
// (eps = eps_rel x the input bounding-box diagonal; the feature tube and the anchor ball are half
// of it). Membership tests use the input itself: a winding number above 1/2 is inside, as in the
// hybrid filter, and a point within eps of a surface counts as on it either way.

namespace {

struct InputMesh
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
};

InputMesh load_welded(const std::string& path)
{
    Eigen::MatrixXd V0;
    Eigen::MatrixXi F0;
    REQUIRE(igl::read_triangle_mesh(path, V0, F0));
    InputMesh m;
    Eigen::VectorXi SVI, SVJ;
    igl::remove_duplicate_vertices(V0, F0, 0.0, m.V, SVI, SVJ, m.F);
    return m;
}

double bbox_diag(const std::vector<const Eigen::MatrixXd*>& Vs)
{
    Eigen::RowVector3d lo = Eigen::RowVector3d::Constant(std::numeric_limits<double>::max());
    Eigen::RowVector3d hi = -lo;
    for (const auto* V : Vs) {
        lo = lo.cwiseMin(V->colwise().minCoeff());
        hi = hi.cwiseMax(V->colwise().maxCoeff());
    }
    return (hi - lo).norm();
}

// Squared distances from each row of P to the mesh.
Eigen::VectorXd sq_dist(const Eigen::MatrixXd& P, const InputMesh& m)
{
    Eigen::VectorXd D;
    Eigen::VectorXi I;
    Eigen::MatrixXd C;
    igl::point_mesh_squared_distance(P, m.V, m.F, D, I, C);
    return D;
}

// The manifold edges whose two face normals are more than 60 degrees apart, as an edge mesh.
void sharp_edges(
    const InputMesh& m,
    std::vector<Vector3d>& ev,
    std::vector<std::array<size_t, 2>>& ee)
{
    Eigen::MatrixXd N;
    igl::per_face_normals(m.V, m.F, N);
    Eigen::MatrixXi E, uE;
    Eigen::VectorXi EMAP;
    std::vector<std::vector<int>> uE2E;
    igl::unique_edge_map(m.F, E, uE, EMAP, uE2E);
    std::map<int, size_t> vid;
    const int nf = int(m.F.rows());
    for (int u = 0; u < uE.rows(); ++u) {
        if (uE2E[u].size() != 2) continue;
        const int f0 = uE2E[u][0] % nf, f1 = uE2E[u][1] % nf;
        if (N.row(f0).dot(N.row(f1)) >= 0.5) continue; // cos 60
        std::array<size_t, 2> e;
        for (int k = 0; k < 2; ++k) {
            const int v = uE(u, k);
            const auto [it, inserted] = vid.emplace(v, ev.size());
            if (inserted) ev.push_back(m.V.row(v).transpose());
            e[k] = it->second;
        }
        ee.push_back(e);
    }
}

void write_edges_obj(
    const fs::path& p,
    const std::vector<Vector3d>& ev,
    const std::vector<std::array<size_t, 2>>& ee)
{
    std::ofstream o(p);
    o.precision(17);
    for (const auto& v : ev) o << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
    for (const auto& e : ee) o << "l " << e[0] + 1 << " " << e[1] + 1 << "\n";
}

void write_points_obj(const fs::path& p, const std::vector<Vector3d>& pts)
{
    std::ofstream o(p);
    o.precision(17);
    for (const auto& v : pts) o << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
}

struct HybridBlocks
{
    std::vector<Vector3d> tet_v, face_v, edge_v, points;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<std::array<size_t, 3>> faces;
    std::vector<std::array<size_t, 2>> edges;
};

HybridBlocks read_hybrid(const fs::path& file)
{
    HybridBlocks h;
    MshData msh;
    msh.load(file.string());
    h.tet_v.resize(msh.get_num_tet_vertices());
    msh.extract_tet_vertices(
        [&](size_t i, double x, double y, double z) { h.tet_v[i] << x, y, z; });
    h.tets.resize(msh.get_num_tets());
    msh.extract_tets(
        [&](size_t i, size_t a, size_t b, size_t c, size_t e) { h.tets[i] = {{a, b, c, e}}; });
    h.face_v.resize(msh.get_num_face_vertices());
    msh.extract_face_vertices(
        [&](size_t i, double x, double y, double z) { h.face_v[i] << x, y, z; });
    h.faces.resize(msh.get_num_faces());
    msh.extract_faces([&](size_t i, size_t a, size_t b, size_t c) { h.faces[i] = {{a, b, c}}; });
    h.edge_v.resize(msh.get_num_edge_vertices());
    msh.extract_edge_vertices(
        [&](size_t i, double x, double y, double z) { h.edge_v[i] << x, y, z; });
    h.edges.resize(msh.get_num_edges());
    msh.extract_edges([&](size_t i, size_t a, size_t b) { h.edges[i] = {{a, b}}; });
    const mshio::MshSpec spec = mshio::load_msh(file.string());
    for (const auto& eb : spec.elements.entity_blocks) {
        if (eb.element_type != 15) continue;
        for (const auto& nb : spec.nodes.entity_blocks) {
            if (nb.entity_dim != 0 || nb.entity_tag != eb.entity_tag) continue;
            for (size_t i = 0; i < nb.num_nodes_in_block; ++i) {
                h.points.emplace_back(nb.data[3 * i], nb.data[3 * i + 1], nb.data[3 * i + 2]);
            }
        }
    }
    return h;
}

Eigen::MatrixXd rows(const std::vector<Vector3d>& v)
{
    Eigen::MatrixXd M(v.size(), 3);
    for (size_t i = 0; i < v.size(); ++i) M.row(i) = v[i].transpose();
    return M;
}

// Number of tets whose centre is outside every volume: winding number at most 1/2 with respect
// to each, and farther than eps from each one's surface.
size_t tets_outside_volumes(
    const HybridBlocks& h,
    const std::vector<const InputMesh*>& volumes,
    double eps)
{
    std::vector<Vector3d> centres;
    for (const auto& t : h.tets) {
        centres.push_back((h.tet_v[t[0]] + h.tet_v[t[1]] + h.tet_v[t[2]] + h.tet_v[t[3]]) / 4);
    }
    const Eigen::MatrixXd C = rows(centres);
    std::vector<char> inside(centres.size(), 0);
    for (const InputMesh* m : volumes) {
        const utils::WindingNumberHierarchy wnh(m->V, m->F);
        const Eigen::VectorXd D = sq_dist(C, *m);
        for (size_t i = 0; i < centres.size(); ++i) {
            if (wnh.winding_number(C.row(i)) > 0.5 || D[i] <= eps * eps) inside[i] = 1;
        }
    }
    return size_t(std::count(inside.begin(), inside.end(), 0));
}

} // namespace

TEST_CASE("hybrid-thingi-volume-curves-points", "[tetwild][hybrid][thingi][.]")
{
    // Thingi10K 104509, closed: a volume whose 2492 sharp edges are given as feature curves and
    // four of whose own vertices are given as feature points (a forced point on an input
    // vertex, VolumeRemesher #31's case).
    const fs::path d = fs::temp_directory_path() / "wmtk_test_hybrid_thingi_curves";
    fs::create_directories(d);
    const std::string model = std::string(WMTK_DATA_DIR) + "/models/challenging_tetwild_104509.stl";
    const InputMesh body = load_welded(model);

    std::vector<Vector3d> ev;
    std::vector<std::array<size_t, 2>> ee;
    sharp_edges(body, ev, ee);
    REQUIRE(ee.size() > 100);
    write_edges_obj(d / "sharp.obj", ev, ee);
    std::vector<Vector3d> pts;
    {
        Eigen::Index i;
        body.V.col(0).minCoeff(&i);
        pts.push_back(body.V.row(i).transpose());
        body.V.col(0).maxCoeff(&i);
        pts.push_back(body.V.row(i).transpose());
        body.V.col(1).minCoeff(&i);
        pts.push_back(body.V.row(i).transpose());
        body.V.col(2).maxCoeff(&i);
        pts.push_back(body.V.row(i).transpose());
    }
    write_points_obj(d / "points.obj", pts);

    const nlohmann::json j = {
        {"application", "tetwild"},
        {"input", {model}},
        {"input_roles", {"volume"}},
        {"input_edges", {(d / "sharp.obj").string()}},
        {"input_points", {(d / "points.obj").string()}},
        {"filter", "hybrid"},
        {"num_threads", 0},
        {"output", (d / "out").string()}};
    REQUIRE_NOTHROW(components::tetwild::tetwild(j));
    const HybridBlocks h = read_hybrid(d / "out_hybrid.msh");
    const double eps = 1e-3 * bbox_diag({&body.V});

    // Tets: inside the body.
    REQUIRE(!h.tets.empty());
    CHECK(tets_outside_volumes(h, {&body}, eps) == 0);
    // No surface-role input, so no triangle block.
    CHECK(h.faces.empty());

    // Lines: every vertex in the feature tube around the input curves -- the containment tetwild
    // guarantees. Not the total length, and not coverage of the input by the output: the engine
    // reports coverage as a diagnostic and never gates on it, and on this model 2056 of the
    // 8399 units of sharp edge lie within the tube of another, non-adjacent sharp edge (faces
    // narrower than the tube), so those curves legally merge and the output is 16% shorter while
    // still covering 99.95% of the input within the tube.
    REQUIRE(!h.edges.empty());
    double worst = 0;
    for (const Vector3d& p : h.edge_v) {
        double best = std::numeric_limits<double>::infinity();
        for (const auto& e : ee)
            best = std::min(best, point_segment_distance(p, ev[e[0]], ev[e[1]]));
        worst = std::max(worst, best);
    }
    CHECK(worst <= 0.5 * eps);

    // Points: each input point has an anchor within its ball.
    for (const Vector3d& p : pts) {
        double best = std::numeric_limits<double>::infinity();
        for (const Vector3d& q : h.points) best = std::min(best, (q - p).norm());
        CHECK(best <= 0.5 * eps);
    }
}

TEST_CASE("hybrid-thingi-volume-and-open-surface", "[tetwild][hybrid][thingi][.]")
{
    // Thingi10K 1047980 (a closed plate, volume role) and 103197 (open: 636 open edges, surface
    // role), overlapping in space: the sheet passes through the plate's region.
    const fs::path d = fs::temp_directory_path() / "wmtk_test_hybrid_thingi_sheet";
    fs::create_directories(d);
    const std::string vol = std::string(WMTK_DATA_DIR) + "/models/challenging_tetwild_1047980.stl";
    const std::string sur = std::string(WMTK_DATA_DIR) + "/models/challenging_tetwild_103197.stl";
    const InputMesh plate = load_welded(vol);
    const InputMesh sheet = load_welded(sur);

    const nlohmann::json j = {
        {"application", "tetwild"},
        {"input", {vol, sur}},
        {"input_roles", {"volume", "surface"}},
        {"filter", "hybrid"},
        {"num_threads", 0},
        {"output", (d / "out").string()}};
    REQUIRE_NOTHROW(components::tetwild::tetwild(j));
    const HybridBlocks h = read_hybrid(d / "out_hybrid.msh");
    const double eps = 1e-3 * bbox_diag({&plate.V, &sheet.V});

    // Tets: inside the plate (the only volume-role input).
    REQUIRE(!h.tets.empty());
    CHECK(tets_outside_volumes(h, {&plate}, eps) == 0);

    // Triangles: on the surface-role input (every vertex within eps of it) and not inside the
    // plate (each centre outside it or within eps of its surface).
    REQUIRE(!h.faces.empty());
    const Eigen::VectorXd Dv = sq_dist(rows(h.face_v), sheet);
    CHECK(Dv.maxCoeff() <= eps * eps);
    std::vector<Vector3d> fc;
    for (const auto& f : h.faces)
        fc.push_back((h.face_v[f[0]] + h.face_v[f[1]] + h.face_v[f[2]]) / 3);
    const Eigen::MatrixXd FC = rows(fc);
    const utils::WindingNumberHierarchy wnh(plate.V, plate.F);
    const Eigen::VectorXd Dp = sq_dist(FC, plate);
    size_t in_plate = 0;
    for (Eigen::Index i = 0; i < FC.rows(); ++i) {
        if (wnh.winding_number(FC.row(i)) > 0.5 && Dp[i] > eps * eps) ++in_plate;
    }
    CHECK(in_plate == 0);
    // No curves or points were given.
    CHECK(h.edges.empty());
    CHECK(h.points.empty());
}
