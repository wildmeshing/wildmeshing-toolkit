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
#include <wmtk/utils/predicates.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
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

// ---- Connected scenes: the feature demos ------------------------------------------------------
//
// TetWild meshes every input into one tet mesh, so where a sheet meets the volume, or a curve
// enters it or lies on a sheet, the meeting points are shared mesh vertices. _hybrid.msh writes
// each block with its own vertex array, all copied from the mesh's double positions, so a shared
// vertex appears in every block that uses it with bit-identical coordinates. "Connected" is
// checked that way: exact coordinate equality between blocks.
//
// The scenes are the feature demos the hybrid output was built on (hybrid-data/feature_demo2
// and feature_demo4), all on the cube [0,2]^3:
//   demo2: a cantilever sheet at z = 1 sticking out of the x = 2 face (x 2..3.5, y 0.7..1.3); a
//          helix rod leaving the y = 0 face; a point inside the cube and one in free space.
//   demo4: sheets S1 (out of the x = 2 face, x 2..3.2, y 0.4..1.6) and S2 (out of the y = 0
//          face, y -1.2..0, x 0.4..1.6), both at z = 1; one wire that starts on S2, enters the
//          cube through y = 0, loops inside, leaves through x = 2, runs as an arc in S1,
//          re-enters, loops again, leaves through y = 0 and ends on S2.

namespace {

using Key = std::array<double, 3>;
Key key_of(const Vector3d& p)
{
    return {{p[0], p[1], p[2]}};
}

std::set<Key> vertex_set(const std::vector<Vector3d>& V)
{
    std::set<Key> s;
    for (const auto& p : V) s.insert(key_of(p));
    return s;
}

// The used vertices of an element block, as positions.
template <size_t N>
std::vector<Vector3d> used_vertices(
    const std::vector<Vector3d>& V,
    const std::vector<std::array<size_t, N>>& elements)
{
    std::set<size_t> ids;
    for (const auto& e : elements) ids.insert(e.begin(), e.end());
    std::vector<Vector3d> out;
    for (const size_t i : ids) out.push_back(V[i]);
    return out;
}

fs::path data_model(const std::string& name)
{
    return fs::path(WMTK_DATA_DIR) / "models" / name;
}

// A point is on the closed cube [0,2]^3 up to eps.
bool in_cube(const Vector3d& p, double eps)
{
    for (int k = 0; k < 3; ++k) {
        if (p[k] < -eps || p[k] > 2 + eps) return false;
    }
    return true;
}

} // namespace

TEST_CASE("hybrid-demo2-cantilever-and-rod", "[tetwild][hybrid][demo][.]")
{
    const fs::path d = fs::temp_directory_path() / "wmtk_test_hybrid_demo2";
    fs::create_directories(d);
    const nlohmann::json j = {
        {"application", "tetwild"},
        {"input",
         {data_model("hybrid_demo_cube.obj").string(),
          data_model("hybrid_demo2_sheet.obj").string()}},
        {"input_roles", {"volume", "surface"}},
        {"input_edges", {data_model("hybrid_demo2_helix.obj").string()}},
        {"input_points", {data_model("hybrid_demo2_points.obj").string()}},
        {"filter", "hybrid"},
        {"num_threads", 0},
        {"stop_energy", 10},
        {"max_iterations", 12},
        {"output", (d / "out").string()}};
    REQUIRE_NOTHROW(components::tetwild::tetwild(j));
    const HybridBlocks h = read_hybrid(d / "out_hybrid.msh");
    // Input bounding box: x 0..3.5 (the sheet), y -1.2..2 (the helix), z 0..2.
    const double eps = 1e-3 * Vector3d(3.5, 3.2, 2.0).norm();
    REQUIRE(!h.tets.empty());
    REQUIRE(!h.faces.empty());
    REQUIRE(!h.edges.empty());
    const std::set<Key> tet_keys = vertex_set(used_vertices(h.tet_v, h.tets));

    // The sheet is attached: the sheet vertices that are also tet vertices -- the shared ones --
    // lie on the x = 2 face and span the sheet's attached edge, y 0.7..1.3.
    double y_lo = 10, y_hi = -10;
    size_t shared = 0;
    for (const Vector3d& p : used_vertices(h.face_v, h.faces)) {
        if (tet_keys.count(key_of(p)) == 0) continue;
        ++shared;
        CHECK(std::abs(p.x() - 2) <= eps);
        y_lo = std::min(y_lo, p.y());
        y_hi = std::max(y_hi, p.y());
    }
    CHECK(shared >= 2);
    CHECK(std::abs(y_lo - 0.7) <= eps);
    CHECK(std::abs(y_hi - 1.3) <= eps);

    // The rod is attached: its end on the y = 0 face, (1.35, 0, 1), is a vertex of the line block
    // within the anchor ball, and that vertex is a tet vertex.
    const Vector3d rod_end(1.35, 0, 1);
    size_t attached = 0;
    for (const Vector3d& p : used_vertices(h.edge_v, h.edges)) {
        if ((p - rod_end).norm() <= 0.5 * eps && tet_keys.count(key_of(p)) == 1) ++attached;
    }
    CHECK(attached == 1);

    // The point inside the cube is a tet vertex; both points are anchors within their balls.
    for (const Vector3d& q : {Vector3d(0.5, 0.5, 0.5), Vector3d(3.2, 1.0, 1.35)}) {
        double best = std::numeric_limits<double>::infinity();
        Vector3d at;
        for (const Vector3d& p : h.points) {
            if ((p - q).norm() < best) {
                best = (p - q).norm();
                at = p;
            }
        }
        CHECK(best <= 0.5 * eps);
        if (in_cube(q, 0)) CHECK(tet_keys.count(key_of(at)) == 1);
    }
}

TEST_CASE("hybrid-demo4-two-sheets-and-a-wire", "[tetwild][hybrid][demo][.]")
{
    const fs::path d = fs::temp_directory_path() / "wmtk_test_hybrid_demo4";
    fs::create_directories(d);
    const nlohmann::json j = {
        {"application", "tetwild"},
        {"input",
         {data_model("hybrid_demo_cube.obj").string(),
          data_model("hybrid_demo4_sheet_S1.obj").string(),
          data_model("hybrid_demo4_sheet_S2.obj").string()}},
        {"input_roles", {"volume", "surface", "surface"}},
        {"input_edges", {data_model("hybrid_demo4_wire.obj").string()}},
        {"filter", "hybrid"},
        {"num_threads", 0},
        {"stop_energy", 10},
        {"max_iterations", 15},
        {"output", (d / "out").string()}};
    REQUIRE_NOTHROW(components::tetwild::tetwild(j));
    const HybridBlocks h = read_hybrid(d / "out_hybrid.msh");
    // Input bounding box: x 0..3.2 (S1), y -1.2..2 (S2), z 0..2.
    const double eps = 1e-3 * Vector3d(3.2, 3.2, 2.0).norm();
    REQUIRE(!h.tets.empty());
    REQUIRE(!h.faces.empty());
    REQUIRE(!h.edges.empty());
    const std::set<Key> tet_keys = vertex_set(used_vertices(h.tet_v, h.tets));
    const std::vector<Vector3d> sheet_v = used_vertices(h.face_v, h.faces);
    const std::set<Key> sheet_keys = vertex_set(sheet_v);

    // Both sheets are attached: the sheet vertices that are also tet vertices lie on the face
    // their sheet leaves from (S1: x = 2, S2: y = 0), and each sheet has some.
    size_t on_x2 = 0, on_y0 = 0;
    for (const Vector3d& p : sheet_v) {
        if (tet_keys.count(key_of(p)) == 0) continue;
        const bool s1 = std::abs(p.x() - 2) <= eps;
        const bool s2 = std::abs(p.y()) <= eps;
        CHECK((s1 || s2));
        on_x2 += s1;
        on_y0 += s2;
    }
    CHECK(on_x2 >= 2);
    CHECK(on_y0 >= 2);

    // The wire is shared wherever the input wire is in the cube or on a sheet: each of its vertices
    // is a tet vertex or a sheet vertex, except along the input's free-space segments -- the
    // input edges with an endpoint outside the cube and off the sheets' plane z = 1 (the wire
    // leaves the x = 2 face just below S1 and climbs onto it) -- within the feature tube of them.
    std::vector<Vector3d> wv;
    std::vector<std::array<size_t, 2>> we;
    {
        std::ifstream in(data_model("hybrid_demo4_wire.obj"));
        std::string tag;
        while (in >> tag) {
            if (tag == "v") {
                Vector3d p;
                in >> p[0] >> p[1] >> p[2];
                wv.push_back(p);
            } else if (tag == "l") {
                std::string line;
                std::getline(in, line);
                std::istringstream ls(line);
                std::vector<size_t> ids;
                size_t i;
                while (ls >> i) ids.push_back(i - 1);
                for (size_t k = 0; k + 1 < ids.size(); ++k) we.push_back({{ids[k], ids[k + 1]}});
            } else {
                std::string rest;
                std::getline(in, rest);
            }
        }
    }
    const auto free_in_input = [&](const Vector3d& p) { return !in_cube(p, 0) && p.z() != 1.0; };
    size_t unshared_outside_free = 0, on_sheet = 0;
    for (const Vector3d& p : used_vertices(h.edge_v, h.edges)) {
        const bool tet = tet_keys.count(key_of(p)) == 1;
        const bool sheet = sheet_keys.count(key_of(p)) == 1;
        if (sheet && !in_cube(p, eps)) ++on_sheet;
        if (tet || sheet) continue;
        bool near_free = false;
        for (const auto& e : we) {
            if ((free_in_input(wv[e[0]]) || free_in_input(wv[e[1]])) &&
                point_segment_distance(p, wv[e[0]], wv[e[1]]) <= 0.5 * eps) {
                near_free = true;
                break;
            }
        }
        if (!near_free) ++unshared_outside_free;
    }
    CHECK(unshared_outside_free == 0);
    CHECK(on_sheet > 0); // the arc in S1 and the ends on S2
}

// ---- Connected scenes on Thingi10K models (hidden: minutes each) -----------------------------
//
// A solid (volume role), an open sheet (surface role) placed to cut it, and a wire (input_edges)
// through both, with its free end and its branch point also given as input_points. The wire is a
// trefoil knot -- a closed loop through the solid's walls and through the sheet -- plus a branch
// from one of the knot's vertices: a straight run to the sheet, a stretch ALONG the sheet on the
// sheet's own mesh edges (so it lies on the sheet exactly), and a straight run off the sheet,
// along its normal, to a free end. The same checks run on both scenes, every bound taken from
//
//   eps    eps_rel (1e-3) x the diagonal of the bounding box of every input, the wire included:
//          the envelope every tracked face stays within, exactly;
//   eps/2  the feature tube every tagged edge stays within, and the anchor ball.
//
// A point is inside the solid when its winding number is above 1/2 AND it is farther than eps
// from the solid's surface, outside when the winding number is at most 1/2 AND it is farther
// than eps; the shell between is neither, since there the output may put a point either side.

namespace {

// An edge mesh with, per edge, whether it lies on the sheet: an edge of one of the sheet's
// triangles, between two of the sheet's own vertices.
struct Wire
{
    std::vector<Vector3d> v;
    std::vector<std::array<size_t, 2>> e;
    std::vector<char> on_sheet;
    size_t branch = 0, free_end = 0;

    size_t add(const Vector3d& p)
    {
        v.push_back(p);
        return v.size() - 1;
    }
    void edge(size_t a, size_t b, bool sheet)
    {
        e.push_back({{a, b}});
        on_sheet.push_back(sheet);
    }
    // A straight run off the sheet from vertex a to the point q, in pieces no longer than step,
    // ending on q exactly (p + (q - p) need not round to q). Returns q's vertex.
    size_t run(size_t a, const Vector3d& q, double step)
    {
        const Vector3d p = v[a];
        const int n = std::max(1, int(std::ceil((q - p).norm() / step)));
        for (int s = 1; s <= n; ++s) {
            const size_t b = add(s == n ? q : Vector3d(p + (q - p) * (double(s) / n)));
            edge(a, b, false);
            a = b;
        }
        return a;
    }
};

void write_mesh_obj(const fs::path& p, const InputMesh& m)
{
    std::ofstream o(p);
    o.precision(17);
    for (Eigen::Index i = 0; i < m.V.rows(); ++i)
        o << "v " << m.V(i, 0) << " " << m.V(i, 1) << " " << m.V(i, 2) << "\n";
    for (Eigen::Index i = 0; i < m.F.rows(); ++i)
        o << "f " << m.F(i, 0) + 1 << " " << m.F(i, 1) + 1 << " " << m.F(i, 2) + 1 << "\n";
}

// The trefoil (sin t + 2 sin 2t, cos t - 2 cos 2t, -sin 3t), scaled about c by sxy in x and y
// and by sz in z, as a closed polygon of n vertices.
std::vector<Vector3d> trefoil(const Vector3d& c, double sxy, double sz, int n)
{
    const double pi = std::acos(-1.0);
    std::vector<Vector3d> p;
    for (int i = 0; i < n; ++i) {
        const double t = 2 * pi * i / n;
        p.push_back(
            c + Vector3d(
                    sxy * (std::sin(t) + 2 * std::sin(2 * t)),
                    sxy * (std::cos(t) - 2 * std::cos(2 * t)),
                    -sz * std::sin(3 * t)));
    }
    return p;
}

// Area-weighted normal of m at vertex vid.
Vector3d vertex_normal(const InputMesh& m, int vid)
{
    Vector3d n = Vector3d::Zero();
    for (Eigen::Index f = 0; f < m.F.rows(); ++f) {
        if (m.F(f, 0) != vid && m.F(f, 1) != vid && m.F(f, 2) != vid) continue;
        const Vector3d a = m.V.row(m.F(f, 0)).transpose();
        const Vector3d b = m.V.row(m.F(f, 1)).transpose();
        const Vector3d c = m.V.row(m.F(f, 2)).transpose();
        n += (b - a).cross(c - a);
    }
    return n.normalized();
}

std::vector<std::vector<int>> vertex_adjacency(const InputMesh& m)
{
    std::vector<std::vector<int>> adj(m.V.rows());
    for (Eigen::Index f = 0; f < m.F.rows(); ++f) {
        for (int k = 0; k < 3; ++k) {
            adj[m.F(f, k)].push_back(m.F(f, (k + 1) % 3));
            adj[m.F(f, (k + 1) % 3)].push_back(m.F(f, k));
        }
    }
    return adj;
}

// The vertices of m's largest edge-connected piece.
std::vector<int> largest_piece(const InputMesh& m)
{
    const auto adj = vertex_adjacency(m);
    std::vector<int> label(m.V.rows(), -1);
    std::vector<std::vector<int>> pieces;
    for (int s = 0; s < int(m.V.rows()); ++s) {
        if (label[s] >= 0 || adj[s].empty()) continue;
        pieces.emplace_back();
        std::vector<int> stack = {s};
        label[s] = int(pieces.size()) - 1;
        while (!stack.empty()) {
            const int u = stack.back();
            stack.pop_back();
            pieces.back().push_back(u);
            for (const int x : adj[u]) {
                if (label[x] < 0) {
                    label[x] = label[s];
                    stack.push_back(x);
                }
            }
        }
    }
    return *std::max_element(pieces.begin(), pieces.end(), [](const auto& a, const auto& b) {
        return a.size() < b.size();
    });
}

int nearest_vertex(const InputMesh& m, const std::vector<int>& among, const Vector3d& p)
{
    int best = among.front();
    for (const int v : among) {
        if ((m.V.row(v).transpose() - p).squaredNorm() <
            (m.V.row(best).transpose() - p).squaredNorm())
            best = v;
    }
    return best;
}

// The shortest path from vertex `from` to vertex `to` along m's edges, edge length as weight
// (Dijkstra), as vertex ids; empty if they are not connected.
std::vector<int> shortest_edge_path(const InputMesh& m, int from, int to)
{
    const auto adj = vertex_adjacency(m);
    std::vector<double> dist(m.V.rows(), std::numeric_limits<double>::infinity());
    std::vector<int> prev(m.V.rows(), -1);
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
    dist[from] = 0;
    q.push({0, from});
    while (!q.empty()) {
        const auto [du, u] = q.top();
        q.pop();
        if (du > dist[u]) continue;
        if (u == to) break;
        for (const int x : adj[u]) {
            const double dx = du + (m.V.row(u) - m.V.row(x)).norm();
            if (dx < dist[x]) {
                dist[x] = dx;
                prev[x] = u;
                q.push({dx, x});
            }
        }
    }
    if (!std::isfinite(dist[to])) return {};
    std::vector<int> path = {to};
    while (path.back() != from) path.push_back(prev[path.back()]);
    std::reverse(path.begin(), path.end());
    return path;
}

// The wire: the closed loop, a run from loop[branch] to the first stretch vertex, the stretch
// along the sheet's vertices, and a run of tail_length along the sheet's normal at the last
// stretch vertex, to the free end. Runs are cut into pieces no longer than the loop's edges.
Wire make_wire(
    const std::vector<Vector3d>& loop,
    size_t branch,
    const InputMesh& sheet,
    const std::vector<int>& stretch,
    double tail_length)
{
    Wire w;
    for (const Vector3d& p : loop) w.add(p);
    for (size_t i = 0; i < loop.size(); ++i) w.edge(i, (i + 1) % loop.size(), false);
    const double step = (loop[1] - loop[0]).norm();
    w.branch = branch;
    size_t a = w.run(branch, sheet.V.row(stretch.front()).transpose(), step);
    for (size_t k = 1; k < stretch.size(); ++k) {
        const size_t b = w.add(sheet.V.row(stretch[k]).transpose());
        w.edge(a, b, true);
        a = b;
    }
    w.free_end = w.run(a, w.v[a] + tail_length * vertex_normal(sheet, stretch.back()), step);
    return w;
}

struct Crossing
{
    Vector3d at;
    double sin_angle; // between the wire edge and the triangle's plane
};

// Where the wire's edges that `use` selects cross m's triangles: the edge's endpoints strictly on
// opposite sides of the triangle's plane, and its line through the closed triangle, both decided
// by exact orientation tests. A crossing through a triangle edge is found from both triangles;
// the checks below ask something of each crossing, so a repeat changes nothing.
template <typename Use>
std::vector<Crossing> wire_crossings(const Wire& w, const Use& use, const InputMesh& m)
{
    using utils::predicates::orient3d;
    using O = utils::predicates::Orientation;
    std::vector<Crossing> out;
    for (size_t i = 0; i < w.e.size(); ++i) {
        if (!use(i)) continue;
        const Vector3d& p = w.v[w.e[i][0]];
        const Vector3d& q = w.v[w.e[i][1]];
        for (Eigen::Index f = 0; f < m.F.rows(); ++f) {
            const Vector3d a = m.V.row(m.F(f, 0)).transpose();
            const Vector3d b = m.V.row(m.F(f, 1)).transpose();
            const Vector3d c = m.V.row(m.F(f, 2)).transpose();
            const O sp = orient3d(a, b, c, p), sq = orient3d(a, b, c, q);
            if (sp == O::COPLANAR || sq == O::COPLANAR || sp == sq) continue;
            const O s[3] = {orient3d(p, q, a, b), orient3d(p, q, b, c), orient3d(p, q, c, a)};
            const bool pos = std::count(s, s + 3, O::POSITIVE) > 0;
            const bool neg = std::count(s, s + 3, O::NEGATIVE) > 0;
            if (pos && neg) continue;
            const Vector3d n = (b - a).cross(c - a);
            const double vp = n.dot(p - a), vq = n.dot(q - a);
            out.push_back(
                {p + (vp / (vp - vq)) * (q - p),
                 std::abs(n.normalized().dot((q - p).normalized()))});
        }
    }
    return out;
}

// The vertices of the tet block's boundary faces (faces of exactly one tet).
std::set<Key> tet_boundary_vertices(const HybridBlocks& h)
{
    std::map<std::array<size_t, 3>, int> incident;
    for (const auto& t : h.tets) {
        for (int skip = 0; skip < 4; ++skip) {
            std::array<size_t, 3> f;
            for (int j = 0, k = 0; j < 4; ++j) {
                if (j != skip) f[k++] = t[j];
            }
            std::sort(f.begin(), f.end());
            ++incident[f];
        }
    }
    std::set<Key> out;
    for (const auto& [f, n] : incident) {
        if (n != 1) continue;
        for (const size_t v : f) out.insert(key_of(h.tet_v[v]));
    }
    return out;
}

// Connected pieces of the line block.
size_t line_pieces(const HybridBlocks& h)
{
    std::vector<size_t> root(h.edge_v.size());
    for (size_t i = 0; i < root.size(); ++i) root[i] = i;
    const auto find = [&](size_t x) {
        while (root[x] != x) x = root[x] = root[root[x]];
        return x;
    };
    std::set<size_t> used;
    for (const auto& e : h.edges) {
        root[find(e[0])] = find(e[1]);
        used.insert(e.begin(), e.end());
    }
    std::set<size_t> roots;
    for (const size_t v : used) roots.insert(find(v));
    return roots.size();
}

void check_connected_scene(
    const fs::path& d,
    const std::string& solid_path,
    const InputMesh& solid,
    const InputMesh& sheet,
    const Wire& w)
{
    write_mesh_obj(d / "sheet.obj", sheet);
    write_edges_obj(d / "wire.obj", w.v, w.e);
    const std::vector<Vector3d> input_points = {w.v[w.free_end], w.v[w.branch]};
    write_points_obj(d / "points.obj", input_points);
    const std::string sheet_path = (d / "sheet.obj").string();
    const nlohmann::json j = {
        {"application", "tetwild"},
        {"input", {solid_path, sheet_path}},
        {"input_roles", {"volume", "surface"}},
        {"input_edges", {(d / "wire.obj").string()}},
        {"input_points", {(d / "points.obj").string()}},
        {"filter", "hybrid"},
        {"num_threads", 0},
        {"output", (d / "out").string()}};
    REQUIRE_NOTHROW(components::tetwild::tetwild(j));
    const HybridBlocks h = read_hybrid(d / "out_hybrid.msh");
    const Eigen::MatrixXd WV = rows(w.v);
    const double eps = 1e-3 * bbox_diag({&solid.V, &sheet.V, &WV});

    REQUIRE(!h.tets.empty());
    REQUIRE(!h.faces.empty());
    REQUIRE(!h.edges.empty());
    const std::vector<Vector3d> tet_v = used_vertices(h.tet_v, h.tets);
    const std::vector<Vector3d> sheet_v = used_vertices(h.face_v, h.faces);
    const std::vector<Vector3d> line_v = used_vertices(h.edge_v, h.edges);
    const std::set<Key> tet_keys = vertex_set(tet_v), sheet_keys = vertex_set(sheet_v),
                        line_keys = vertex_set(line_v);
    const utils::WindingNumberHierarchy wnh(solid.V, solid.F);
    const auto winding = [&](const Vector3d& p) {
        return wnh.winding_number(Eigen::RowVector3d(p.transpose()));
    };
    const Eigen::VectorXd line_d2_solid = sq_dist(rows(line_v), solid);

    // Tets: inside the solid.
    CHECK(tets_outside_volumes(h, {&solid}, eps) == 0);

    // Triangles: on the sheet (every vertex within eps of it) and not inside the solid (each
    // centre outside it or within eps of its surface).
    const Eigen::VectorXd sheet_d2_sheet = sq_dist(rows(sheet_v), sheet);
    INFO(
        "sheet vertex farthest from the input sheet: " << std::sqrt(sheet_d2_sheet.maxCoeff()) / eps
                                                       << " eps");
    CHECK(sheet_d2_sheet.maxCoeff() <= eps * eps);
    {
        std::vector<Vector3d> fc;
        for (const auto& f : h.faces)
            fc.push_back((h.face_v[f[0]] + h.face_v[f[1]] + h.face_v[f[2]]) / 3);
        const Eigen::VectorXd D = sq_dist(rows(fc), solid);
        size_t in_solid = 0;
        for (size_t i = 0; i < fc.size(); ++i) {
            if (winding(fc[i]) > 0.5 && D[i] > eps * eps) ++in_solid;
        }
        CHECK(in_solid == 0);
    }

    // Lines: every vertex in the feature tube around the input wire.
    {
        double worst = 0;
        for (const Vector3d& p : line_v) {
            double best = std::numeric_limits<double>::infinity();
            for (const auto& e : w.e)
                best = std::min(best, point_segment_distance(p, w.v[e[0]], w.v[e[1]]));
            worst = std::max(worst, best);
        }
        INFO("line vertex farthest from the input wire: " << worst / eps << " eps");
        CHECK(worst <= 0.5 * eps);
    }

    // 1. The sheet meets the solid: the sheet vertices that are also tet vertices -- where the
    // sheet's tracked faces outside the solid meet the kept tets -- lie within eps of the solid's
    // surface (the kept region ends at tracked faces lying on the solid) and of the sheet (they
    // are corners of sheet triangles); and there are some, since the sheet crosses the solid's
    // surface and its tracked faces on either side share their vertices there.
    {
        std::vector<Vector3d> shared;
        for (const Vector3d& p : sheet_v) {
            if (tet_keys.count(key_of(p))) shared.push_back(p);
        }
        REQUIRE(!shared.empty());
        const Eigen::VectorXd Ds = sq_dist(rows(shared), solid);
        const Eigen::VectorXd Dh = sq_dist(rows(shared), sheet);
        INFO(
            shared.size() << " sheet vertices are tet vertices; farthest from the solid "
                          << std::sqrt(Ds.maxCoeff()) / eps << " eps, from the sheet "
                          << std::sqrt(Dh.maxCoeff()) / eps << " eps");
        CHECK(Ds.maxCoeff() <= eps * eps);
        CHECK(Dh.maxCoeff() <= eps * eps);
    }

    // 2. The wire is in the solid's tets where it is inside the solid: every line vertex inside
    // it is a tet vertex. The tagged edges are edges of the tet mesh, and the kept region is
    // everything the tracked surface on the solid encloses, which contains every point inside
    // (the tracked surface stays within eps of the solid's).
    {
        size_t inside = 0, missed = 0;
        for (size_t i = 0; i < line_v.size(); ++i) {
            if (winding(line_v[i]) <= 0.5 || line_d2_solid[i] <= eps * eps) continue;
            ++inside;
            if (!tet_keys.count(key_of(line_v[i]))) ++missed;
        }
        INFO(inside << " line vertices inside the solid");
        CHECK(missed == 0);
    }

    // 3. The wire is on the sheet where it runs along it: every line vertex in the tube of the
    // stretch, outside the solid, and not in the tube of any off-sheet input edge, is a sheet
    // vertex. The stretch's input edges are edges of the sheet's triangles, so the insertion
    // makes them edges of tracked faces, and no operation separates the two: a collapse renames
    // the tagged edges and the tracked faces at a vertex together, a split of such an edge
    // splits its faces, and a swap keeps tracked faces. Outside the solid no kept tet touches
    // the vertex, so its tracked faces are sheet triangles. Excluded: the shell around the
    // solid, where the vertex may be a tet vertex instead, and the tubes of the runs that join
    // the stretch at its ends, whose own vertices need not be on the sheet. Not guaranteed: that
    // the stretch keeps an interior vertex at all (it is several target edge lengths long here).
    {
        size_t checked = 0, missed = 0;
        for (size_t i = 0; i < line_v.size(); ++i) {
            const Vector3d& p = line_v[i];
            double on = std::numeric_limits<double>::infinity(), off = on;
            for (size_t k = 0; k < w.e.size(); ++k) {
                double& to = w.on_sheet[k] ? on : off;
                to = std::min(to, point_segment_distance(p, w.v[w.e[k][0]], w.v[w.e[k][1]]));
            }
            if (on > 0.5 * eps || off <= 0.5 * eps) continue;
            if (winding(p) > 0.5 || line_d2_solid[i] <= eps * eps) continue;
            ++checked;
            if (!sheet_keys.count(key_of(p))) ++missed;
        }
        INFO(checked << " line vertices on the stretch outside the solid");
        CHECK(checked > 0);
        CHECK(missed == 0);
    }

    // 4. The wire is connected where it crosses the solid's surface and where it pierces the
    // sheet. The tagged edges, the tet faces where the kept region ends and the sheet triangles
    // are all faces of ONE tet mesh, so a tagged path can pass from one side of a tracked
    // surface to the other only through a mesh vertex on it: at a crossing of the solid's
    // surface, a vertex of the kept region's boundary faces; at a piercing of the sheet outside
    // the solid, a sheet vertex. Where is that vertex? Within eps/2 of the wire (the tube) and
    // within eps of the surface crossed (the envelope). Where the wire is a straight line through
    // c meeting a plane at angle alpha, such a point x = c + t u + r, |r| <= eps/2, has
    // |t| sin(alpha) <= eps + eps/2, so |x - c| <= reach(c) = 1.5 eps / sin(alpha) + eps/2. This
    // assumes the output curve still passes the crossing (coverage, which the pipeline reports
    // but does not gate) and the surface is flat across reach(c). A piercing is checked only
    // outside the solid by more than reach(c) + eps, where the sheet vertex cannot be a tet vertex
    // and its triangles are all output; one inside the solid is check 2's.
    {
        const std::set<Key> boundary = tet_boundary_vertices(h);
        const auto reach = [&](const Crossing& c) { return 1.5 * eps / c.sin_angle + 0.5 * eps; };
        const auto nearest = [&](const Vector3d& c, const std::set<Key>& in) {
            double best = std::numeric_limits<double>::infinity();
            for (const Vector3d& p : line_v) {
                if (in.count(key_of(p))) best = std::min(best, (p - c).norm());
            }
            return best;
        };

        const std::vector<Crossing> through_solid =
            wire_crossings(w, [](size_t) { return true; }, solid);
        REQUIRE(through_solid.size() >= 2);
        size_t missed = 0;
        double worst = 0; // nearest shared vertex over reach, the largest
        for (const Crossing& c : through_solid) {
            const double r = nearest(c.at, boundary) / reach(c);
            worst = std::max(worst, r);
            if (r > 1) ++missed;
        }
        INFO(
            through_solid.size() << " crossings of the solid's surface; nearest boundary line "
                                    "vertex at most "
                                 << worst << " x reach");
        CHECK(missed == 0);

        const std::vector<Crossing> through_sheet =
            wire_crossings(w, [&](size_t i) { return !w.on_sheet[i]; }, sheet);
        std::vector<Vector3d> at;
        for (const Crossing& c : through_sheet) at.push_back(c.at);
        const Eigen::VectorXd D = sq_dist(rows(at), solid);
        size_t checked = 0;
        missed = 0;
        worst = 0;
        for (size_t i = 0; i < through_sheet.size(); ++i) {
            const Crossing& c = through_sheet[i];
            const double r = reach(c);
            if (winding(c.at) > 0.5 || D[i] <= (r + eps) * (r + eps)) continue;
            ++checked;
            const double ratio = nearest(c.at, sheet_keys) / r;
            worst = std::max(worst, ratio);
            if (ratio > 1) ++missed;
        }
        REQUIRE(checked >= 1);
        INFO(
            through_sheet.size() << " piercings of the sheet, " << checked
                                 << " outside the solid; nearest sheet line vertex at most "
                                 << worst << " x reach");
        CHECK(missed == 0);
    }

    // 5. Points. Each input point has an anchor within its ball (the anchor guarantee), and that
    // anchor is a line vertex: the vertex carrying an anchor passes it on only to the survivor of
    // its collapse, which takes over its tagged edges. The line block is one connected piece, as
    // the input wire is: the tagged edges change only by splits and by collapses (which rename one
    // endpoint to the other, merging edges that coincide), never by deletion -- a swap that would
    // remove one is refused, a collapse that would lose one is rolled back -- and renaming
    // vertices cannot disconnect a graph. NOT guaranteed, so only reported: the branch point's
    // degree. With allow_junction_cleanup on (the default), a collapse may rename a vertex of one
    // branch to a vertex of another within the tube, merging their edges, so the junction can
    // move along the curves and its anchor can be left with fewer (or more) than three edges.
    {
        std::map<Key, size_t> degree;
        for (const auto& e : h.edges) {
            ++degree[key_of(h.edge_v[e[0]])];
            ++degree[key_of(h.edge_v[e[1]])];
        }
        for (const Vector3d& q : input_points) {
            double best = std::numeric_limits<double>::infinity();
            Vector3d anchor;
            for (const Vector3d& p : h.points) {
                if ((p - q).norm() < best) {
                    best = (p - q).norm();
                    anchor = p;
                }
            }
            INFO(
                "anchor " << best / eps << " eps from the input point, with "
                          << degree[key_of(anchor)] << " line edges");
            CHECK(best <= 0.5 * eps);
            CHECK(line_keys.count(key_of(anchor)) == 1);
        }
        CHECK(line_pieces(h) == 1);
    }
}

} // namespace

TEST_CASE("hybrid-thingi-solid-wavy-sheet-wire", "[tetwild][hybrid][thingi][.]")
{
    // Thingi10K 104509, closed: a frame of walls about 1 thick (eps is 0.195 here). The sheet is
    // z = c_z + a sin(k (x - c_x)) sin(k (y - c_y)) about the solid's box centre c, a = ext_z / 4
    // and k = 3 pi / max(ext_x, ext_y) (1.5 periods across the solid), on a 40 x 40 grid over
    // the solid's xy box grown by 10% of its extent on each side: it cuts the walls along wavy
    // lines. The loop is the trefoil about c, scaled by ext_x / 10 in x, y and ext_z / 6 in z: it
    // crosses the walls 6 times and the sheet 6 times, outside the solid. The stretch is the
    // grid's column 15 (x = -18.2) from row 4 to 13 (y = -34.0 to -8.1): it goes through the
    // wall at y = -19.96..-18.96 inside the sheet. The branch is the loop vertex nearest the
    // stretch's start, and the free end is 0.1 x the solid's diagonal off the sheet.
    const fs::path d = fs::temp_directory_path() / "wmtk_test_hybrid_thingi_wavy_sheet";
    fs::create_directories(d);
    const std::string solid_path = data_model("challenging_tetwild_104509.stl").string();
    const InputMesh solid = load_welded(solid_path);
    const Vector3d lo = solid.V.colwise().minCoeff().transpose();
    const Vector3d hi = solid.V.colwise().maxCoeff().transpose();
    const Vector3d ext = hi - lo, c = (lo + hi) / 2;

    const int n = 40;
    const double pi = std::acos(-1.0);
    const double a = ext.z() / 4, k = 3 * pi / std::max(ext.x(), ext.y());
    InputMesh sheet;
    sheet.V.resize((n + 1) * (n + 1), 3);
    sheet.F.resize(2 * n * n, 3);
    for (int row = 0; row <= n; ++row) {
        for (int col = 0; col <= n; ++col) {
            const double x = lo.x() - 0.1 * ext.x() + 1.2 * ext.x() * col / n;
            const double y = lo.y() - 0.1 * ext.y() + 1.2 * ext.y() * row / n;
            sheet.V.row(row * (n + 1) + col) << x, y,
                c.z() + a * std::sin(k * (x - c.x())) * std::sin(k * (y - c.y()));
        }
    }
    for (int row = 0; row < n; ++row) {
        for (int col = 0; col < n; ++col) {
            const int v0 = row * (n + 1) + col, v1 = v0 + 1, v2 = v0 + n + 2, v3 = v0 + n + 1;
            sheet.F.row(2 * (row * n + col)) << v0, v1, v2;
            sheet.F.row(2 * (row * n + col) + 1) << v0, v2, v3;
        }
    }

    const std::vector<Vector3d> loop = trefoil(c, ext.x() / 10, ext.z() / 6, 120);
    std::vector<int> stretch;
    for (int row = 4; row <= 13; ++row) stretch.push_back(row * (n + 1) + 15);
    size_t branch = 0;
    for (size_t i = 0; i < loop.size(); ++i) {
        if ((loop[i] - sheet.V.row(stretch[0]).transpose()).norm() <
            (loop[branch] - sheet.V.row(stretch[0]).transpose()).norm())
            branch = i;
    }
    const Wire w = make_wire(loop, branch, sheet, stretch, 0.1 * ext.norm());
    check_connected_scene(d, solid_path, solid, sheet, w);
}

TEST_CASE("hybrid-thingi-solid-open-surface-wire", "[tetwild][hybrid][thingi][.]")
{
    // Thingi10K 1047980, closed: a ring of radius 40, wall 2 thick, z = -5.1..5.1. The sheet is
    // Thingi10K 103197 (open, 6 pieces), scaled by 0.8 and lowered by 10.4: its band z = 10..16,
    // where its surfaces stand at radius 20..50, moves to the ring's height, so the ring's wall
    // cuts it along several curves. The loop is the trefoil about the ring's centre, scaled by
    // ext_x / 5 in x, y and 0.3 ext_z in z: it goes through the wall 6 times and through the
    // sheet some 30 times. The branch is loop vertex 80, in the ring's hole; the stretch is the
    // shortest path along the sheet's edges, on its largest piece, from the vertex nearest the
    // branch to the vertex nearest (33, 25, 7), outside the ring above its top: it goes through
    // the wall inside the sheet. The free end is 0.1 x the ring's diagonal off the sheet.
    const fs::path d = fs::temp_directory_path() / "wmtk_test_hybrid_thingi_open_surface";
    fs::create_directories(d);
    const std::string solid_path = data_model("challenging_tetwild_1047980.stl").string();
    const InputMesh solid = load_welded(solid_path);
    const Vector3d lo = solid.V.colwise().minCoeff().transpose();
    const Vector3d hi = solid.V.colwise().maxCoeff().transpose();
    const Vector3d ext = hi - lo, c = (lo + hi) / 2;

    InputMesh sheet = load_welded(data_model("challenging_tetwild_103197.stl").string());
    for (Eigen::Index i = 0; i < sheet.V.rows(); ++i) {
        sheet.V.row(i) = 0.8 * sheet.V.row(i) + Eigen::RowVector3d(0, 0, -10.4);
    }

    const std::vector<Vector3d> loop = trefoil(c, ext.x() / 5, 0.3 * ext.z(), 120);
    const size_t branch = 80;
    const std::vector<int> piece = largest_piece(sheet);
    const std::vector<int> stretch = shortest_edge_path(
        sheet,
        nearest_vertex(sheet, piece, loop[branch]),
        nearest_vertex(sheet, piece, Vector3d(33, 25, 7)));
    REQUIRE(stretch.size() >= 2);
    const Wire w = make_wire(loop, branch, sheet, stretch, 0.1 * ext.norm());
    check_connected_scene(d, solid_path, solid, sheet, w);
}
