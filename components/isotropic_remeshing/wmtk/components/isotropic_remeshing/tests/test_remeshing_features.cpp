#include <wmtk/components/isotropic_remeshing/IsotropicRemeshing.h>
#include <wmtk/components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk/utils/io.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <string>

using namespace wmtk;
using namespace components::isotropic_remeshing;

namespace {

using Edge = std::array<size_t, 2>;

/// An n x n grid of unit squares in z = 0, two counter-clockwise triangles each; vertex (i, j)
/// at (i, j, 0) is i * (n + 1) + j.
void grid(int n, std::vector<Eigen::Vector3d>& V, std::vector<std::array<size_t, 3>>& F)
{
    const auto id = [n](int i, int j) { return size_t(i * (n + 1) + j); };
    V.clear();
    F.clear();
    for (int i = 0; i <= n; ++i) {
        for (int j = 0; j <= n; ++j) V.emplace_back(i, j, 0);
    }
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            F.push_back({{id(i, j), id(i + 1, j), id(i + 1, j + 1)}});
            F.push_back({{id(i, j), id(i + 1, j + 1), id(i, j + 1)}});
        }
    }
}

/// The live vertex at `p`, or -1.
int64_t vertex_at(const IsotropicRemeshing& m, const Eigen::Vector3d& p)
{
    for (const auto& t : m.get_vertices()) {
        if (m.vertex_attrs.at(t.vid(m)).pos == p) return int64_t(t.vid(m));
    }
    return -1;
}

/// Walks the curve from its end `from` and returns the ids of its edges in order; requires it
/// to be a chain that reaches `to`.
std::vector<int64_t> walk(const IsotropicRemeshing& m, size_t from, size_t to)
{
    std::vector<int64_t> ids;
    size_t prev = size_t(-1), cur = from;
    while (cur != to) {
        const auto& f = m.feature_attrs.at(cur);
        REQUIRE((f.nbrs.size() == 2 || (cur == from && f.nbrs.size() == 1)));
        const size_t next = f.nbrs[0] != prev ? f.nbrs[0] : f.nbrs[1];
        ids.push_back(m.feature_edge_id(cur, next));
        REQUIRE(m.feature_edge_id(next, cur) == ids.back());
        prev = cur;
        cur = next;
        REQUIRE(ids.size() <= m.feature_edges().size());
    }
    return ids;
}

/// The grid of size 8 with a curve along y = 4: ids 7 left of x = 4 and 9 right of it, so
/// (4, 4) is where the id changes; its ends, a vertex inside it at (2, 4) and one off it at
/// (6, 2) are fixed, with ids 100 to 103.
struct CurveOnGrid
{
    std::vector<Eigen::Vector3d> V;
    std::vector<std::array<size_t, 3>> F;
    std::vector<Edge> edges;
    std::vector<int64_t> ids;
    std::vector<std::pair<size_t, int64_t>> fixed;
    CurveOnGrid()
    {
        grid(8, V, F);
        for (int i = 0; i < 8; ++i) {
            edges.push_back({{size_t(i * 9 + 4), size_t((i + 1) * 9 + 4)}});
            ids.push_back(i < 4 ? 7 : 9);
        }
        fixed = {{0 * 9 + 4, 100}, {8 * 9 + 4, 101}, {2 * 9 + 4, 102}, {6 * 9 + 2, 103}};
    }
};

/// The curve of CurveOnGrid: one chain from (0, 4) to (8, 4), ids 7 up to (4, 4) and 9 after
/// it, and the fixed vertices where they were with their ids.
void check_curve_on_grid(const IsotropicRemeshing& m)
{
    for (const auto& [p, id] : std::vector<std::pair<Eigen::Vector3d, int64_t>>{
             {{0, 4, 0}, 100},
             {{8, 4, 0}, 101},
             {{2, 4, 0}, 102},
             {{6, 2, 0}, 103}}) {
        const int64_t v = vertex_at(m, p);
        REQUIRE(v >= 0);
        CHECK(m.is_fixed(v));
        CHECK(m.feature_attrs.at(v).vertex_id == id);
    }
    const int64_t junction = vertex_at(m, {4, 4, 0});
    REQUIRE(junction >= 0);
    const auto left = walk(m, vertex_at(m, {0, 4, 0}), junction);
    const auto right = walk(m, vertex_at(m, {8, 4, 0}), junction);
    CHECK(left == std::vector<int64_t>(left.size(), 7));
    CHECK(right == std::vector<int64_t>(right.size(), 9));
    CHECK(left.size() + right.size() == m.feature_edges().size());
}

/// The DataArray `name` of a .vtu that paraviewo wrote: base64, a UInt64 byte count first.
template <typename T>
std::vector<T> vtu_array(const std::string& path, const std::string& name)
{
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    const std::string s = ss.str();
    const size_t at = s.find("Name=\"" + name + "\"");
    REQUIRE(at != std::string::npos);
    const size_t begin = s.find('>', at) + 1;
    const std::string text = s.substr(begin, s.find("</DataArray>", begin) - begin);
    static const std::string digits =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::vector<unsigned char> bytes;
    uint32_t buffer = 0;
    int bits = 0;
    for (const char c : text) {
        const size_t d = digits.find(c);
        if (d == std::string::npos) continue; // whitespace, padding
        buffer = (buffer << 6) | uint32_t(d);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            bytes.push_back((buffer >> bits) & 0xff);
        }
    }
    REQUIRE(bytes.size() >= 8);
    uint64_t size = 0;
    std::memcpy(&size, bytes.data(), 8);
    REQUIRE(bytes.size() >= 8 + size);
    std::vector<T> out(size / sizeof(T));
    std::memcpy(out.data(), bytes.data() + 8, size);
    return out;
}

struct TempDir
{
    std::filesystem::path path;
    TempDir()
        : path(
              std::filesystem::temp_directory_path() /
              ("wmtk_remeshing_features_" + std::to_string(std::random_device{}())))
    {
        std::filesystem::create_directories(path);
    }
    ~TempDir() { std::filesystem::remove_all(path); }
};

} // namespace

TEST_CASE("features-ids-through-split-and-collapse", "[test_remeshing][features]")
{
    const CurveOnGrid c;
    // Serially, and once with threads.
    for (const int threads : {0, 4}) {
        INFO("threads " << threads);
        IsotropicRemeshing m(c.V, threads);
        m.create_mesh(c.V.size(), c.F);
        m.set_feature_edges(c.edges, c.ids);
        m.set_fixed_vertices(c.fixed);
        check_curve_on_grid(m);

        m.split_remeshing(0.4);
        CHECK(m.feature_edges().size() == 2 * c.edges.size()); // 1 is over 4/3 0.4, 1/2 not
        check_curve_on_grid(m);

        m.collapse_remeshing(3);
        CHECK(m.feature_edges().size() < c.edges.size());
        check_curve_on_grid(m);

        // Whole iterations, and the consolidation renumbering everything.
        m.uniform_remeshing(1.5, 2);
        check_curve_on_grid(m);
    }
}

TEST_CASE("features-fixed-vertices-survive-collapses", "[test_remeshing][features]")
{
    // A fixed vertex off any curve, its eight neighbours pulled in close: every edge to them is
    // short, and the collapse pass removes them into it -- whichever way the queue holds them.
    std::vector<Eigen::Vector3d> V;
    std::vector<std::array<size_t, 3>> F;
    grid(8, V, F);
    const size_t centre = 4 * 9 + 4;
    for (const int di : {-1, 0, 1}) {
        for (const int dj : {-1, 0, 1}) {
            if (di == 0 && dj == 0) continue;
            V[centre + di * 9 + dj] = V[centre] + 0.05 * Eigen::Vector3d(di, dj, 0);
        }
    }
    IsotropicRemeshing m(V, 0);
    m.create_mesh(V.size(), F);
    m.set_fixed_vertices({{centre, 42}});

    // The curve rules refuse to remove a fixed vertex; the other way round is allowed.
    std::vector<IsotropicRemeshing::Tuple> new_tris;
    const auto t = m.tuple_from_edge(centre, centre + 1, m.tuple_from_vertex(centre).fid(m));
    REQUIRE(t.vid(m) == centre);
    CHECK_FALSE(m.may_collapse_away(t));
    CHECK(m.may_collapse_away(t.switch_vertex(m)));
    CHECK_FALSE(m.collapse_edge(t, new_tris));

    m.collapse_remeshing(0.8);
    REQUIRE(m.check_mesh_connectivity_validity());
    REQUIRE(m.vertex_attrs[centre].pos == Eigen::Vector3d(4, 4, 0));
    CHECK(m.is_fixed(centre));
    CHECK(m.feature_attrs.at(centre).vertex_id == 42);
    size_t near = 0;
    for (const auto& v : m.get_vertices()) {
        if ((m.vertex_attrs[v.vid(m)].pos - V[centre]).norm() < 0.5) ++near;
    }
    CHECK(near == 1);

    // Smoothing leaves it.
    m.smooth_all_vertices();
    CHECK(m.vertex_attrs[centre].pos == Eigen::Vector3d(4, 4, 0));
}

TEST_CASE("features-junction-where-ids-change", "[test_remeshing][features]")
{
    // (4, 4) and (5, 4) are both inside the curve, each between two feature edges; at (4, 4) the
    // id changes. Both are moved along the curve, where smoothing would bring them back.
    CurveOnGrid c;
    c.fixed.clear();
    c.V[4 * 9 + 4].x() += 0.3;
    c.V[5 * 9 + 4].x() += 0.3;
    IsotropicRemeshing m(c.V, 0);
    m.create_mesh(c.V.size(), c.F);
    m.set_feature_edges(c.edges, c.ids);
    const size_t junction = 4 * 9 + 4, inside = 5 * 9 + 4;
    CHECK(m.stays_put(junction));
    CHECK_FALSE(m.stays_put(inside));

    m.smooth_all_vertices();
    CHECK(m.vertex_attrs[junction].pos == Eigen::Vector3d(4.3, 4, 0));
    CHECK(m.vertex_attrs[inside].pos.x() < 5.29);

    // A collapse along the curve does not remove it, but may go into it, which keeps its ids
    // on either side.
    // One-ring edge tuples point at the neighbour; switched, at the centre.
    std::optional<IsotropicRemeshing::Tuple> into;
    for (const auto& e : m.get_one_ring_edges_for_vertex(m.tuple_from_vertex(junction))) {
        if (!m.is_feature_edge(junction, e.vid(m))) continue;
        CHECK_FALSE(m.may_collapse_away(e.switch_vertex(m)));
        if (e.vid(m) == inside) into = e;
    }
    REQUIRE(into.has_value());
    REQUIRE(into->vid(m) == inside);
    CHECK(m.may_collapse_away(*into));
    std::vector<IsotropicRemeshing::Tuple> new_tris;
    REQUIRE(m.collapse_edge(*into, new_tris));
    CHECK(m.vertex_attrs[junction].pos == Eigen::Vector3d(4.3, 4, 0));
    REQUIRE(m.feature_degree(junction) == 2);
    const auto& nbrs = m.feature_attrs.at(junction).nbrs;
    std::set<int64_t> ids{
        m.feature_edge_id(junction, nbrs[0]),
        m.feature_edge_id(junction, nbrs[1])};
    CHECK(ids == std::set<int64_t>{7, 9});
}

TEST_CASE("features-seam-kept", "[test_remeshing][features]")
{
    // An open tube with a seam down one side: a curve with the same region -- the same label, if
    // there were labels -- on both sides, which no label boundary would find.
    const int n = 16, rows = 8;
    std::vector<Eigen::Vector3d> V;
    std::vector<std::array<size_t, 3>> F;
    const auto id = [](int k, int j) { return size_t(j * n + (k % n)); };
    for (int j = 0; j <= rows; ++j) {
        for (int k = 0; k < n; ++k) {
            const double a = 2 * M_PI * k / n;
            V.emplace_back(std::cos(a), std::sin(a), 0.25 * j);
        }
    }
    for (int j = 0; j < rows; ++j) {
        for (int k = 0; k < n; ++k) {
            F.push_back({{id(k, j), id(k + 1, j), id(k + 1, j + 1)}});
            F.push_back({{id(k, j), id(k + 1, j + 1), id(k, j + 1)}});
        }
    }
    std::vector<Edge> seam;
    for (int j = 0; j < rows; ++j) seam.push_back({{id(0, j), id(0, j + 1)}});
    for (const int threads : {0}) {
        IsotropicRemeshing m(V, threads);
        m.create_mesh(V.size(), F, {}, true, 0.01);
        m.set_feature_edges(seam, std::vector<int64_t>(seam.size(), 5));
        m.set_fixed_vertices({{id(0, 0), 0}, {id(0, rows), 1}});
        m.set_feature_envelope(0.01);
        m.set_face_fields(std::vector<std::vector<double>>(F.size(), {3.0}));
        m.uniform_remeshing(0.15, 3);
        REQUIRE(m.check_mesh_connectivity_validity());

        // The consolidation renumbered the vertices; the ends are where they were.
        const auto ids = walk(m, vertex_at(m, V[id(0, 0)]), vertex_at(m, V[id(0, rows)]));
        CHECK(ids.size() > seam.size());
        CHECK(ids == std::vector<int64_t>(ids.size(), 5));
        for (const auto& e : m.feature_edges()) {
            // Two triangles at it, as at every edge off the tube's boundary.
            size_t faces = 0;
            for (const auto& f : m.get_one_ring_tris_for_vertex(m.tuple_from_vertex(e[0]))) {
                const auto vs = m.oriented_tri_vids(f);
                faces += std::count(vs.begin(), vs.end(), e[1]);
            }
            CHECK(faces == 2);
            // Still on the line x = 1, y = 0, within the envelope.
            CHECK(std::abs(m.vertex_attrs[e[0]].pos.y()) < 0.01);
        }
        for (const auto& f : m.get_faces()) {
            CHECK(m.face_field_attrs.at(f.fid(m)).values == std::vector<double>{3.0});
        }
    }
}

TEST_CASE("features-file", "[test_remeshing][features]")
{
    // Through the JSON: a strip of two labels, so the label boundary is a curve without an id;
    // a features file with a seam-like curve of id 5 across it, entries of no length, and fixed
    // vertices; a .vtu output.
    const TempDir dir;
    std::vector<Eigen::Vector3d> V;
    std::vector<std::array<size_t, 3>> F;
    grid(6, V, F);
    std::vector<double> label(F.size());
    for (size_t f = 0; f < F.size(); ++f) {
        label[f] = (V[F[f][0]] + V[F[f][1]] + V[F[f][2]]).x() / 3 < 3 ? 0 : 1;
    }
    // In the file, the vertices are in reverse order: not sorted, so that merging duplicates
    // would renumber them.
    const auto r = [&V](size_t v) { return V.size() - 1 - v; };
    {
        MshData msh;
        msh.add_face_vertices(V.size(), [&](size_t i) { return V[r(i)]; });
        msh.add_faces(F.size(), [&](size_t i) {
            return std::array<size_t, 3>{{r(F[i][0]), r(F[i][1]), r(F[i][2])}};
        });
        msh.add_face_attribute<1>("patch_id", [&label](size_t i) { return label[i]; });
        msh.save((dir.path / "in.msh").string());
    }
    // The curve along y = 3, from (0, 3) to (6, 3); it crosses the label boundary x = 3.
    nlohmann::json features;
    for (size_t i = 0; i < 6; ++i) {
        features["edges"].push_back({r(i * 7 + 3), r((i + 1) * 7 + 3)});
        features["edge_ids"].push_back(5);
    }
    features["edges"].push_back({r(10), r(10)}); // no length: skipped
    features["edge_ids"].push_back(6);
    features["vertices"] = {{r(3), 11}, {r(6 * 7 + 3), 12}, {r(1 * 7 + 3), 13}};
    const auto write = [&](const nlohmann::json& j) {
        std::ofstream(dir.path / "features.json") << j.dump();
    };
    write(features);
    nlohmann::json config = {
        {"application", "isotropic_remeshing"},
        {"input", "in.msh"},
        {"input_dir", dir.path.string()},
        {"output", (dir.path / "out.vtu").string()},
        {"feature_fields", {"patch_id"}},
        {"features", "features.json"},
        {"remove_duplicate_eps", -1},
        {"length_abs", 0.7},
        {"eps_rel", 0.01},
        {"max_iterations", 2}};

    SECTION("output")
    {
        isotropic_remeshing(config);
        const std::string tri = (dir.path / "out.vtu").string();
        const std::string lines = (dir.path / "out_edges.vtu").string();
        REQUIRE(std::filesystem::exists(tri));
        REQUIRE(std::filesystem::exists(lines));
        const auto points = vtu_array<double>(tri, "vertex_id"); // a point field
        const auto patch = vtu_array<double>(tri, "patch_id");
        const auto conn = vtu_array<int64_t>(tri, "connectivity");
        const auto edge_id = vtu_array<double>(lines, "edge_id");
        const auto edge_conn = vtu_array<int64_t>(lines, "connectivity");
        CHECK(vtu_array<double>(lines, "vertex_id") == points);
        REQUIRE(conn.size() == 3 * patch.size());
        REQUIRE(edge_conn.size() == 2 * edge_id.size());

        std::multiset<int64_t> fixed;
        for (const double v : points) {
            if (v >= 0) fixed.insert(int64_t(v));
        }
        CHECK(fixed == std::multiset<int64_t>{11, 12, 13});
        for (const double p : patch) CHECK((p == 0 || p == 1));
        // Each line is an edge of the triangles; the curve keeps id 5, the label boundary -1,
        // and the patch changes exactly across the lines of id -1.
        std::map<Edge, std::vector<size_t>> faces;
        for (size_t f = 0; f < patch.size(); ++f) {
            for (int k = 0; k < 3; ++k) {
                size_t a = conn[3 * f + k], b = conn[3 * f + (k + 1) % 3];
                faces[{{std::min(a, b), std::max(a, b)}}].push_back(f);
            }
        }
        std::map<Edge, int64_t> line_id;
        for (size_t i = 0; i < edge_id.size(); ++i) {
            const size_t a = edge_conn[2 * i], b = edge_conn[2 * i + 1];
            line_id[{{std::min(a, b), std::max(a, b)}}] = int64_t(edge_id[i]);
            CHECK((edge_id[i] == 5 || edge_id[i] == -1));
        }
        size_t curve = 0;
        for (const auto& [e, fs] : faces) {
            REQUIRE(fs.size() <= 2);
            const bool differ = fs.size() == 2 && patch[fs[0]] != patch[fs[1]];
            const auto it = line_id.find(e);
            CHECK(differ == (it != line_id.end() && it->second == -1));
            if (it != line_id.end() && it->second == 5) ++curve;
        }
        CHECK(curve == size_t(std::count(edge_id.begin(), edge_id.end(), 5.)));
        CHECK(curve >= 6);
    }
    SECTION("an edge not in the mesh")
    {
        features["edges"].push_back({r(0), r(48)});
        features["edge_ids"].push_back(8);
        write(features);
        CHECK_THROWS(isotropic_remeshing(config));
    }
    SECTION("vertices renumbered")
    {
        // Merging duplicates sorts the vertices: the indices would no longer name them.
        config["remove_duplicate_eps"] = 1e-5;
        CHECK_THROWS(isotropic_remeshing(config));
    }
}

namespace {
double smallest_angle(const IsotropicRemeshing& m)
{
    double a = 180;
    for (const auto& f : m.get_faces()) {
        const auto vs = m.oriented_tri_vids(f);
        for (int k = 0; k < 3; ++k) {
            const Eigen::Vector3d u =
                m.vertex_attrs[vs[(k + 1) % 3]].pos - m.vertex_attrs[vs[k]].pos;
            const Eigen::Vector3d w =
                m.vertex_attrs[vs[(k + 2) % 3]].pos - m.vertex_attrs[vs[k]].pos;
            a = std::min(a, std::atan2(u.cross(w).norm(), u.dot(w)) * 180 / M_PI);
        }
    }
    return a;
}

size_t facing_down(const IsotropicRemeshing& m)
{
    size_t n = 0;
    for (const auto& f : m.get_faces()) {
        const auto vs = m.oriented_tri_vids(f);
        const auto& p = m.vertex_attrs;
        if ((p[vs[1]].pos - p[vs[0]].pos).cross(p[vs[2]].pos - p[vs[0]].pos).z() <= 0) ++n;
    }
    return n;
}
} // namespace

TEST_CASE("cleanup-repairs-thin-and-folded-triangles", "[test_remeshing][features]")
{
    // The curve of CurveOnGrid, with a vertex next to it pulled almost onto it -- thin
    // triangles between a curve and a vertex off it, which no swap may mend and the curve
    // vertices may not leave -- and another pushed past its neighbour, folding its star over.
    CurveOnGrid c;
    c.V[3 * 9 + 5] = Eigen::Vector3d(3.1, 4.05, 0);
    c.V[6 * 9 + 6] = Eigen::Vector3d(7.3, 6.2, 0);
    for (const bool cleanup : {false, true}) {
        INFO("cleanup " << cleanup);
        IsotropicRemeshing m(c.V, 0);
        if (cleanup) m.set_cleanup_angle(30);
        m.create_mesh(c.V.size(), c.F, {}, true, 0.1);
        m.set_feature_edges(c.edges, c.ids);
        m.set_fixed_vertices(c.fixed);
        m.set_feature_envelope(0.1);
        const double before = smallest_angle(m);
        REQUIRE(before < 3);
        REQUIRE(facing_down(m) > 0);
        if (cleanup) {
            CHECK(m.cleanup_triangles(30) > 0);
            CHECK(smallest_angle(m) > 20);
            CHECK(facing_down(m) == 0);
        }
        REQUIRE(m.check_mesh_connectivity_validity());
        // The curve, its ids and the fixed vertices hold, and it stays on y = 4.
        check_curve_on_grid(m);
        for (const auto& e : m.feature_edges()) CHECK(m.vertex_attrs[e[0]].pos.y() == 4);
    }
}
