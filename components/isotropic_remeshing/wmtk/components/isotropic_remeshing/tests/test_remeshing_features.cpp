#include <igl/read_triangle_mesh.h>
#include <wmtk/components/isotropic_remeshing/IsotropicRemeshing.h>
#include <wmtk/components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/history/Replay.hpp>
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/io.hpp>

#include <spdlog/sinks/ostream_sink.h>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <string>

// Cases too slow for the Debug CI jobs, Windows Debug above all: Release builds only.
#ifdef NDEBUG
const std::string release_only = "";
#else
const std::string release_only = "[.]";
#endif

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

/// The DataArray `name` of a .vtu that paraviewo wrote -- "Points" for the points, whose array
/// has no name: base64, a UInt64 byte count first.
template <typename T>
std::vector<T> vtu_array(const std::string& path, const std::string& name)
{
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    const std::string s = ss.str();
    const size_t at = name == "Points" ? s.find("<DataArray", s.find("<Points>"))
                                       : s.find("Name=\"" + name + "\"");
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
        // A label each side of the curve, which is their boundary: every face keeps the label of
        // its side, however the threads split the work.
        const auto side = [&m](const IsotropicRemeshing::Tuple& f) {
            const auto vs = m.oriented_tri_vids(f);
            double y = 0;
            for (const size_t v : vs) y += m.vertex_attrs.at(v).pos.y() / 3;
            return y > 4 ? 1. : 0.;
        };
        {
            std::vector<std::vector<double>> labels;
            for (const auto& f : m.get_faces()) labels.push_back({side(f)});
            m.set_face_fields(labels);
        }
        const auto check_labels = [&]() {
            size_t wrong = 0;
            for (const auto& f : m.get_faces()) {
                wrong += m.face_field_attrs.at(f.fid(m)).values != std::vector<double>{side(f)};
            }
            CHECK(wrong == 0);
        };
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
        check_labels();
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
    // The edge to centre + 1, in a triangle that has it: not every triangle around the centre does.
    std::optional<size_t> fid;
    for (const auto& g : m.get_one_ring_tris_for_vertex(m.tuple_from_vertex(centre))) {
        const auto vs = m.oriented_tri_vids(g);
        if (std::find(vs.begin(), vs.end(), centre + 1) != vs.end()) fid = g.fid(m);
    }
    REQUIRE(fid.has_value());
    const auto t = m.tuple_from_edge(centre, centre + 1, *fid);
    REQUIRE(t.vid(m) == centre);
    CHECK_FALSE(m.may_collapse_away(t));
    CHECK(m.may_collapse_away(t.switch_vertex(m)));
    CHECK_FALSE(m.collapse_edge(t, new_tris));

    m.collapse_remeshing(0.8);
    REQUIRE(m.check_mesh_connectivity_validity());
    REQUIRE(m.vertex_attrs[centre].pos == Eigen::Vector3d(4, 4, 0));
    CHECK(m.is_fixed(centre));
    CHECK(m.feature_attrs.at(centre).vertex_id == 42);
    size_t nearby = 0;
    for (const auto& v : m.get_vertices()) {
        if ((m.vertex_attrs[v.vid(m)].pos - V[centre]).norm() < 0.5) ++nearby;
    }
    CHECK(nearby == 1);

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

TEST_CASE("features-tube-keeps-a-bent-curve", "[test_remeshing][features]" + release_only)
{
    // An L-shaped curve in a flat grid, its corner at (4, 4) inside the curve: smoothing slides
    // the corner towards its neighbours' midpoint, and a collapse along the curve leaves a chord
    // across it, both within the plane, so the surface envelope holds nothing. Only the tube
    // keeps the curve where it was.
    std::vector<Eigen::Vector3d> V;
    std::vector<std::array<size_t, 3>> F;
    grid(8, V, F);
    std::vector<Edge> curve;
    for (int i = 0; i < 4; ++i) curve.push_back({{size_t(i * 9 + 4), size_t((i + 1) * 9 + 4)}});
    for (int j = 4; j < 8; ++j) curve.push_back({{size_t(4 * 9 + j), size_t(4 * 9 + j + 1)}});
    // How far `p` is from the L: (0, 4) to (4, 4) to (4, 8).
    const auto off = [](const Eigen::Vector3d& p) {
        const double along_x = std::hypot(p.x() - std::clamp(p.x(), 0., 4.), p.y() - 4);
        const double along_y = std::hypot(p.x() - 4, p.y() - std::clamp(p.y(), 4., 8.));
        return std::min(along_x, along_y);
    };
    const double eps = 0.01;
    for (const bool tube : {false, true}) {
        INFO("tube " << tube);
        IsotropicRemeshing m(V, 0);
        m.create_mesh(V.size(), F, {}, true, eps);
        m.set_feature_edges(curve);
        if (tube) m.set_feature_envelope(eps);
        m.uniform_remeshing(1.2, 3);
        REQUIRE(m.check_mesh_connectivity_validity());
        double far = 0;
        for (const auto& e : m.feature_edges()) {
            far = std::max({far, off(m.vertex_attrs[e[0]].pos), off(m.vertex_attrs[e[1]].pos)});
        }
        if (tube) {
            CHECK(far <= eps);
        } else {
            CHECK(far > 0.1);
        }
    }
}

TEST_CASE("features-seam-kept", "[test_remeshing][features]" + release_only)
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

TEST_CASE("features-file", "[test_remeshing][features]" + release_only)
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
        // Merging duplicates (remove_duplicate_eps 1e-5, the default) sorts the vertices even
        // where none coincide; the features follow them.
        for (const double merge : {-1., 1e-5}) {
            INFO("remove_duplicate_eps " << merge);
            config["remove_duplicate_eps"] = merge;
            isotropic_remeshing(config);
            const std::string tri = (dir.path / "out.vtu").string();
            const std::string lines = (dir.path / "out_edge.vtu").string();
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
    }

    SECTION("an .obj output")
    {
        // It has no place for the ids, and says so.
        config["output"] = (dir.path / "out.obj").string();
        std::ostringstream log;
        const auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(log);
        logger().sinks().push_back(sink);
        try {
            isotropic_remeshing(config);
        } catch (...) {
            logger().sinks().pop_back();
            throw;
        }
        logger().sinks().pop_back();
        CHECK(log.str().find("holds the triangles only") != std::string::npos);
        CHECK(std::filesystem::exists(dir.path / "out.obj"));
        CHECK_FALSE(std::filesystem::exists(dir.path / "out_edge.vtu"));
    }
    SECTION("an edge not in the mesh")
    {
        features["edges"].push_back({r(0), r(48)});
        features["edge_ids"].push_back(8);
        write(features);
        CHECK_THROWS(isotropic_remeshing(config));
    }
    SECTION("not integers")
    {
        // Each a clear error, rather than a truncated id or the JSON library's own.
        const auto refused = [&](const nlohmann::json& j, const std::string& error) {
            write(j);
            CHECK_THROWS_WITH(
                isotropic_remeshing(config),
                Catch::Matchers::ContainsSubstring(error));
        };
        nlohmann::json j = features;
        j["edge_ids"][0] = 5.7;
        refused(j, "edge id 5.7 in");
        j["edge_ids"][0] = "5";
        refused(j, "edge id \"5\" in");
        j = features;
        j["vertices"][0][1] = 1.5;
        refused(j, "vertex id 1.5 in");
        j = features;
        j["edges"][0][1] = 3.0;
        refused(j, "edge end 3.0 in");
        j = features;
        j["vertices"] = 3;
        refused(j, "vertices in");
        refused(features["edges"], "is not an object");
    }
}

TEST_CASE("features-follow-merged-vertices", "[test_remeshing][features]" + release_only)
{
    // The grid of size 6 in two pieces, x <= 3 and x >= 3, each with its own copy of the column
    // x = 3, as a B-rep mesher writes a mesh face by face. The features list the seam x = 3 on
    // both copies, its end (3, 0) fixed on both, and an edge between the two copies of (3, 6):
    // merging the duplicates welds the pieces, the two copies of the seam into one curve, and
    // takes the edge between the copies away.
    const TempDir dir;
    std::vector<Eigen::Vector3d> V;
    std::vector<std::array<size_t, 3>> F;
    const auto left = [](int i, int j) { return size_t(i * 7 + j); }; // i in [0, 3]
    const auto right = [](int i, int j) { return size_t(28 + (i - 3) * 7 + j); }; // i in [3, 6]
    for (int i = 0; i <= 3; ++i) {
        for (int j = 0; j <= 6; ++j) V.emplace_back(i, j, 0);
    }
    for (int i = 3; i <= 6; ++i) {
        for (int j = 0; j <= 6; ++j) V.emplace_back(i, j, 0);
    }
    for (int i = 0; i < 6; ++i) {
        const auto id = [&](int a, int b) { return i < 3 ? left(a, b) : right(a, b); };
        for (int j = 0; j < 6; ++j) {
            F.push_back({{id(i, j), id(i + 1, j), id(i + 1, j + 1)}});
            F.push_back({{id(i, j), id(i + 1, j + 1), id(i, j + 1)}});
        }
    }
    {
        MshData msh;
        msh.add_face_vertices(V.size(), [&](size_t i) { return V[i]; });
        msh.add_faces(F.size(), [&](size_t i) { return F[i]; });
        msh.save((dir.path / "in.msh").string());
    }
    nlohmann::json features;
    for (int j = 0; j < 6; ++j) {
        features["edges"].push_back({left(3, j), left(3, j + 1)});
        features["edges"].push_back({right(3, j + 1), right(3, j)});
        features["edge_ids"].push_back(5);
        features["edge_ids"].push_back(5);
    }
    features["edges"].push_back({left(3, 6), right(3, 6)});
    features["edge_ids"].push_back(7);
    features["vertices"] = {{left(3, 0), 11}, {right(3, 0), 11}, {left(0, 0), 12}};
    const auto write = [&](const nlohmann::json& j) {
        std::ofstream(dir.path / "features.json") << j.dump();
    };
    const nlohmann::json config = {
        {"application", "isotropic_remeshing"},
        {"input", "in.msh"},
        {"input_dir", dir.path.string()},
        {"output", (dir.path / "out.vtu").string()},
        {"features", "features.json"},
        {"length_abs", 0.7},
        {"eps_rel", 0.01},
        {"max_iterations", 2}};

    SECTION("welded")
    {
        write(features);
        isotropic_remeshing(config);
        const std::string tri = (dir.path / "out.vtu").string();
        const std::string lines = (dir.path / "out_edge.vtu").string();
        std::multiset<int64_t> fixed;
        for (const double v : vtu_array<double>(tri, "vertex_id")) {
            if (v >= 0) fixed.insert(int64_t(v));
        }
        CHECK(fixed == std::multiset<int64_t>{11, 12});
        const auto edge_id = vtu_array<double>(lines, "edge_id");
        CHECK(edge_id.size() >= 6);
        CHECK(size_t(std::count(edge_id.begin(), edge_id.end(), 5.)) == edge_id.size());
        // One curve inside the surface: two triangles at each of its edges.
        const auto conn = vtu_array<int64_t>(tri, "connectivity");
        const auto edge_conn = vtu_array<int64_t>(lines, "connectivity");
        std::map<Edge, size_t> faces;
        for (size_t f = 0; 3 * f < conn.size(); ++f) {
            for (int k = 0; k < 3; ++k) {
                const size_t a = conn[3 * f + k], b = conn[3 * f + (k + 1) % 3];
                ++faces[{{std::min(a, b), std::max(a, b)}}];
            }
        }
        for (size_t i = 0; 2 * i < edge_conn.size(); ++i) {
            const size_t a = edge_conn[2 * i], b = edge_conn[2 * i + 1];
            CHECK(faces[{{std::min(a, b), std::max(a, b)}}] == 2);
        }
    }
    SECTION("ids that disagree")
    {
        // Two copies of one edge, or of one vertex, with different ids.
        nlohmann::json j = features;
        j["edge_ids"][1] = 6;
        write(j);
        CHECK_THROWS_WITH(
            isotropic_remeshing(config),
            Catch::Matchers::ContainsSubstring("are one edge of the input, with ids 5 and 6"));
        j = features;
        j["vertices"][1][1] = 13;
        write(j);
        CHECK_THROWS_WITH(
            isotropic_remeshing(config),
            Catch::Matchers::ContainsSubstring("are one vertex of the input, with ids 11 and 13"));
    }
}

TEST_CASE("features-memory-route", "[test_remeshing][features]" + release_only)
{
    // On the memory route the features are a mesh of the input too, on the rows of the input:
    // the curves its edges, with edge_id, the fixed vertices its points, with vertex_id. The
    // result is what the file route's two .vtu files hold: mesh with vertex_id, and edges, the
    // feature edges on mesh's vertex rows, with edge_id -- which make the next call's input and
    // features.
    const TempDir dir;
    std::vector<Eigen::Vector3d> V;
    std::vector<std::array<size_t, 3>> F;
    grid(6, V, F);
    {
        MshData msh;
        msh.add_face_vertices(V.size(), [&](size_t i) { return V[i]; });
        msh.add_faces(F.size(), [&](size_t i) { return F[i]; });
        msh.save((dir.path / "in.msh").string());
    }
    // The curve along y = 3, ids 5 and 6 either side of x = 3; fixed vertices on it and off it.
    nlohmann::json features;
    for (size_t i = 0; i < 6; ++i) {
        features["edges"].push_back({i * 7 + 3, (i + 1) * 7 + 3});
        features["edge_ids"].push_back(i < 3 ? 5 : 6);
    }
    features["vertices"] = {{3, 11}, {6 * 7 + 3, 12}, {1 * 7 + 3, 13}, {4 * 7 + 1, 14}};
    std::ofstream(dir.path / "features.json") << features.dump();
    const nlohmann::json options = {
        {"length_abs", 0.7},
        {"eps_rel", 0.01},
        {"max_iterations", 2},
        {"swap_quality", true},
        {"limit_collapse_length", true},
        {"cleanup_angle", 30}};

    nlohmann::json file = options;
    file["application"] = "isotropic_remeshing";
    file["input"] = "in.msh";
    file["input_dir"] = dir.path.string();
    file["output"] = (dir.path / "out.vtu").string();
    file["report"] = (dir.path / "report.json").string();
    file["features"] = "features.json";
    isotropic_remeshing(file);

    MeshData surface;
    surface.V.resize(V.size(), 3);
    for (size_t i = 0; i < V.size(); ++i) surface.V.row(i) = V[i];
    auto& tris = surface.cells[CellType::Triangle].connectivity;
    tris.resize(F.size(), 3);
    for (size_t i = 0; i < F.size(); ++i) {
        for (int k = 0; k < 3; ++k) tris(i, k) = int64_t(F[i][k]);
    }
    // The same features as a mesh on the rows of surface.
    MeshData curves;
    curves.V = surface.V;
    {
        auto& edges = curves.cells[CellType::Edge];
        edges.connectivity.resize(features["edges"].size(), 2);
        DataMatrix<int64_t> id(features["edges"].size(), 1);
        for (size_t i = 0; i < features["edges"].size(); ++i) {
            edges.connectivity(i, 0) = features["edges"][i][0];
            edges.connectivity(i, 1) = features["edges"][i][1];
            id(i, 0) = features["edge_ids"][i];
        }
        edges.attributes["edge_id"].values = id;
        auto& points = curves.cells[CellType::Point];
        points.connectivity.resize(features["vertices"].size(), 1);
        DataMatrix<int64_t> vertex_id(features["vertices"].size(), 1);
        for (size_t i = 0; i < features["vertices"].size(); ++i) {
            points.connectivity(i, 0) = features["vertices"][i][0];
            vertex_id(i, 0) = features["vertices"][i][1];
        }
        points.attributes["vertex_id"].values = vertex_id;
    }
    nlohmann::json memory = options;
    memory["input"] = "surface";
    memory["features"] = "curves";
    const auto [meshes, report] =
        isotropic_remeshing(memory, MeshMap{{"surface", surface}, {"curves", curves}});

    REQUIRE(meshes.size() == 2);
    const MeshData& mesh = meshes.at("mesh");
    const MeshData& edges = meshes.at("edges");
    CHECK(edges.V == mesh.V);
    // The ids are Int64, never Float64 on the memory route; the .vtu holds the same values.
    const auto int64 = [](const MeshAttribute& a) {
        return std::holds_alternative<DataMatrix<int64_t>>(a.values);
    };
    CHECK(int64(mesh.vertex_attributes.at("vertex_id")));
    CHECK(int64(edges.vertex_attributes.at("vertex_id")));
    CHECK(int64(edges.cell(CellType::Edge).attributes.at("edge_id")));

    const std::string tri = (dir.path / "out.vtu").string();
    const std::string lines = (dir.path / "out_edge.vtu").string();
    const auto column = [](const MeshAttribute& a) {
        const DataMatrix<double> d = a.doubles();
        return std::vector<double>(d.data(), d.data() + d.size());
    };
    const auto flat = [](const DataMatrix<int64_t>& c) {
        return std::vector<int64_t>(c.data(), c.data() + c.size());
    };
    CHECK(
        flat(mesh.cell(CellType::Triangle).connectivity) ==
        vtu_array<int64_t>(tri, "connectivity"));
    CHECK(column(mesh.vertex_attributes.at("vertex_id")) == vtu_array<double>(tri, "vertex_id"));
    CHECK(
        flat(edges.cell(CellType::Edge).connectivity) == vtu_array<int64_t>(lines, "connectivity"));
    CHECK(
        column(edges.cell(CellType::Edge).attributes.at("edge_id")) ==
        vtu_array<double>(lines, "edge_id"));
    CHECK(column(edges.vertex_attributes.at("vertex_id")) == vtu_array<double>(lines, "vertex_id"));

    // The fixed vertices and the curves' ids, in the result and in what a result makes.
    const auto check_ids = [&](const MeshMap& result) {
        std::multiset<double> ids;
        for (const double v : column(result.at("mesh").vertex_attributes.at("vertex_id"))) {
            if (v >= 0) ids.insert(v);
        }
        CHECK(ids == std::multiset<double>{11, 12, 13, 14});
        const auto edge_id =
            column(result.at("edges").cell(CellType::Edge).attributes.at("edge_id"));
        const auto n5 = std::count(edge_id.begin(), edge_id.end(), 5.);
        const auto n6 = std::count(edge_id.begin(), edge_id.end(), 6.);
        CHECK(n5 >= 3);
        CHECK(n6 >= 3);
        CHECK(size_t(n5 + n6) == edge_id.size());
    };
    check_ids(meshes);

    nlohmann::json file_report;
    std::ifstream(dir.path / "report.json") >> file_report;
    file_report.erase("time_sec");
    nlohmann::json memory_report = report;
    memory_report.erase("time_sec");
    CHECK(memory_report == file_report);

    // A result goes back in as it is: mesh as the input -- its vertex_id is accepted and
    // ignored -- and edges as the features, the fixed vertices from its vertex_id.
    nlohmann::json again = options;
    again["input"] = "mesh";
    again["features"] = "edges";
    check_ids(isotropic_remeshing(again, meshes).first);

    // Fixed vertices without curves: edges still has an edge block, of none.
    MeshData points = curves;
    points.cells.erase(CellType::Edge);
    const MeshData none =
        isotropic_remeshing(memory, MeshMap{{"surface", surface}, {"curves", points}})
            .first.at("edges");
    CHECK(none.cell(CellType::Edge).connectivity.rows() == 0);
    CHECK(none.cell(CellType::Edge).connectivity.cols() == 2);
    CHECK(int64(none.cell(CellType::Edge).attributes.at("edge_id")));

    // What it may not be: on other vertices, with a Float64 id, with triangles.
    const auto refused = [&](const MeshData& wrong, const std::string& error) {
        CHECK_THROWS_WITH(
            isotropic_remeshing(memory, MeshMap{{"surface", surface}, {"curves", wrong}}),
            Catch::Matchers::ContainsSubstring(error));
    };
    MeshData wrong = curves;
    wrong.V.conservativeResize(wrong.V.rows() + 1, 3);
    wrong.V.row(wrong.V.rows() - 1).setZero();
    refused(wrong, "has 50 vertices and the input 49");
    wrong = curves;
    wrong.cells[CellType::Edge].attributes["edge_id"].values = DataMatrix<double>(
        DataMatrix<double>::Constant(wrong.cell(CellType::Edge).connectivity.rows(), 1, 5));
    refused(wrong, "edge_id that is not one Int64 column");
    wrong = curves;
    wrong.cells[CellType::Triangle] = surface.cell(CellType::Triangle);
    refused(wrong, "has cells other than edges and points");
    wrong = curves;
    wrong.cells[CellType::Point].attributes.clear();
    refused(wrong, "Missing required mesh attribute: vertex_id");
}

TEST_CASE("feature-fields-memory-route", "[test_remeshing][features]" + release_only)
{
    // feature_fields on the memory route: triangle attributes of the input mesh, kept as labels as
    // the .msh route keeps the file's, and returned on mesh -- an Int64 one as Int64. A strip of
    // two labels, with a curve of id 5 across it and two fixed vertices.
    const TempDir dir;
    std::vector<Eigen::Vector3d> V;
    std::vector<std::array<size_t, 3>> F;
    grid(6, V, F);
    DataMatrix<int64_t> label(F.size(), 1);
    for (size_t f = 0; f < F.size(); ++f) {
        label(f, 0) = (V[F[f][0]] + V[F[f][1]] + V[F[f][2]]).x() / 3 < 3 ? 0 : 1;
    }
    {
        MshData msh;
        msh.add_face_vertices(V.size(), [&](size_t i) { return V[i]; });
        msh.add_faces(F.size(), [&](size_t i) { return F[i]; });
        msh.add_face_attribute<1>("patch_id", [&label](size_t i) { return double(label(i, 0)); });
        msh.save((dir.path / "in.msh").string());
    }
    nlohmann::json features;
    for (size_t i = 0; i < 6; ++i) {
        features["edges"].push_back({i * 7 + 3, (i + 1) * 7 + 3});
        features["edge_ids"].push_back(5);
    }
    features["vertices"] = {{3, 11}, {6 * 7 + 3, 12}};
    std::ofstream(dir.path / "features.json") << features.dump();
    const nlohmann::json options = {
        {"feature_fields", {"patch_id"}},
        {"length_abs", 0.7},
        {"eps_rel", 0.01},
        {"max_iterations", 2},
        {"cleanup_angle", 30}};

    nlohmann::json file = options;
    file["application"] = "isotropic_remeshing";
    file["input"] = "in.msh";
    file["input_dir"] = dir.path.string();
    file["output"] = (dir.path / "out.vtu").string();
    file["report"] = (dir.path / "report.json").string();
    file["features"] = "features.json";
    isotropic_remeshing(file);

    MeshData surface;
    surface.V.resize(V.size(), 3);
    for (size_t i = 0; i < V.size(); ++i) surface.V.row(i) = V[i];
    auto& tris = surface.cells[CellType::Triangle];
    tris.connectivity.resize(F.size(), 3);
    for (size_t i = 0; i < F.size(); ++i) {
        for (int k = 0; k < 3; ++k) tris.connectivity(i, k) = int64_t(F[i][k]);
    }
    tris.attributes["patch_id"].values = label;
    MeshData curves;
    curves.V = surface.V;
    {
        auto& edges = curves.cells[CellType::Edge];
        edges.connectivity.resize(6, 2);
        for (size_t i = 0; i < 6; ++i) {
            edges.connectivity(i, 0) = features["edges"][i][0];
            edges.connectivity(i, 1) = features["edges"][i][1];
        }
        edges.attributes["edge_id"].values =
            DataMatrix<int64_t>(DataMatrix<int64_t>::Constant(6, 1, 5));
        auto& points = curves.cells[CellType::Point];
        points.connectivity.resize(2, 1);
        points.connectivity << 3, 6 * 7 + 3;
        DataMatrix<int64_t> vertex_id(2, 1);
        vertex_id << 11, 12;
        points.attributes["vertex_id"].values = vertex_id;
    }
    nlohmann::json memory = options;
    memory["input"] = "surface";
    memory["features"] = "curves";
    const auto [meshes, report] =
        isotropic_remeshing(memory, MeshMap{{"surface", surface}, {"curves", curves}});

    // What the two .vtu files hold: the same points, triangles and labels on the same rows --
    // Int64 here -- and the same feature edges, with their ids and the fixed vertices'.
    REQUIRE(meshes.size() == 2);
    const MeshData& mesh = meshes.at("mesh");
    const MeshData& edges = meshes.at("edges");
    const auto& patch = mesh.cell(CellType::Triangle).attributes.at("patch_id");
    REQUIRE(std::holds_alternative<DataMatrix<int64_t>>(patch.values));
    const std::string tri = (dir.path / "out.vtu").string();
    const std::string lines = (dir.path / "out_edge.vtu").string();
    const auto flat = [](const auto& m) {
        using Scalar = typename std::decay_t<decltype(m)>::Scalar;
        return std::vector<Scalar>(m.data(), m.data() + m.size());
    };
    const auto column = [&flat](const MeshAttribute& a) { return flat(a.doubles()); };
    CHECK(flat(mesh.V) == vtu_array<double>(tri, "Points"));
    CHECK(
        flat(mesh.cell(CellType::Triangle).connectivity) ==
        vtu_array<int64_t>(tri, "connectivity"));
    CHECK(column(patch) == vtu_array<double>(tri, "patch_id"));
    CHECK(column(mesh.vertex_attributes.at("vertex_id")) == vtu_array<double>(tri, "vertex_id"));
    CHECK(edges.V == mesh.V);
    CHECK(
        flat(edges.cell(CellType::Edge).connectivity) == vtu_array<int64_t>(lines, "connectivity"));
    CHECK(
        column(edges.cell(CellType::Edge).attributes.at("edge_id")) ==
        vtu_array<double>(lines, "edge_id"));
    CHECK(column(edges.vertex_attributes.at("vertex_id")) == vtu_array<double>(lines, "vertex_id"));

    // Each triangle has the label of its side of x = 3, which is a curve: none crosses it.
    const auto check_labels = [](const MeshData& result) {
        const auto& block = result.cell(CellType::Triangle);
        const DataMatrix<double> label = block.attributes.at("patch_id").doubles();
        REQUIRE(label.rows() == block.connectivity.rows());
        std::set<double> seen;
        for (Eigen::Index f = 0; f < label.rows(); ++f) {
            seen.insert(label(f, 0));
            for (int k = 0; k < 3; ++k) {
                const double x = result.V(block.connectivity(f, k), 0);
                CHECK((label(f, 0) == 0 ? x <= 3 + 1e-9 : x >= 3 - 1e-9));
            }
        }
        CHECK(seen == std::set<double>{0, 1});
    };
    check_labels(mesh);
    nlohmann::json file_report;
    std::ifstream(dir.path / "report.json") >> file_report;
    file_report.erase("time_sec");
    nlohmann::json memory_report = report;
    memory_report.erase("time_sec");
    CHECK(memory_report == file_report);

    // A result goes back in as it is, with the same feature_fields, its labels with it; without
    // them, its patch_id is an attribute the input may not carry.
    nlohmann::json again = options;
    again["input"] = "mesh";
    again["features"] = "edges";
    const auto chained = isotropic_remeshing(again, meshes).first;
    CHECK(
        std::holds_alternative<DataMatrix<int64_t>>(
            chained.at("mesh").cell(CellType::Triangle).attributes.at("patch_id").values));
    check_labels(chained.at("mesh"));
    nlohmann::json bare = again;
    bare["feature_fields"] = nlohmann::json::array();
    CHECK_THROWS_WITH(
        isotropic_remeshing(bare, meshes),
        Catch::Matchers::ContainsSubstring("Unknown mesh attribute: patch_id"));

    // Without features: the label boundary is the only curve, of id -1. A Float64 field comes
    // back Float64.
    MeshData floats = surface;
    floats.cells[CellType::Triangle].attributes["patch_id"].values = label.cast<double>().eval();
    nlohmann::json fields_only = options;
    fields_only["input"] = "surface";
    const auto alone = isotropic_remeshing(fields_only, MeshMap{{"surface", floats}}).first;
    CHECK(
        std::holds_alternative<DataMatrix<double>>(
            alone.at("mesh").cell(CellType::Triangle).attributes.at("patch_id").values));
    check_labels(alone.at("mesh"));
    const auto& edge_id = std::get<DataMatrix<int64_t>>(
        alone.at("edges").cell(CellType::Edge).attributes.at("edge_id").values);
    CHECK(edge_id.rows() >= 6);
    CHECK((edge_id.array() == -1).all());

    // A field feature_fields does not name is still refused, as one it names but the input lacks.
    nlohmann::json unnamed = memory;
    unnamed["feature_fields"] = nlohmann::json::array();
    CHECK_THROWS_WITH(
        isotropic_remeshing(unnamed, MeshMap{{"surface", surface}, {"curves", curves}}),
        Catch::Matchers::ContainsSubstring("Unknown mesh attribute: patch_id"));
    nlohmann::json missing = memory;
    missing["feature_fields"] = {"patch_id", "side"};
    CHECK_THROWS_WITH(
        isotropic_remeshing(missing, MeshMap{{"surface", surface}, {"curves", curves}}),
        Catch::Matchers::ContainsSubstring("has no field side"));
}

TEST_CASE("feature-fields-follow-the-cleaning", "[test_remeshing][features]")
{
    // Small enough for Debug builds, without an envelope: the fields name the faces of the input
    // as given, of which the cleaning drops the degenerate ones -- one that repeats a vertex, and
    // one with two corners that the merging of duplicates welds -- and the values follow the
    // faces it keeps, on both routes. The memory route returns each per-face field in its own
    // type, with its labels: an Int64 label, 2^53 (the largest id it takes) on one side, and
    // UInt8 sides with column labels; and neither carries nor returns a per-corner normal.
    const TempDir dir;
    std::vector<Eigen::Vector3d> V;
    std::vector<std::array<size_t, 3>> F;
    grid(4, V, F);
    constexpr int64_t limit = int64_t(1) << 53;
    V.push_back(V[6]); // a copy of (1, 1), which the merging welds to it
    // First, so that each face of the grid is two rows further down in the input than in the
    // mesh.
    const std::vector<std::array<size_t, 3>> degenerate = {{{5, 6, V.size() - 1}}, {{7, 7, 8}}};
    F.insert(F.begin(), degenerate.begin(), degenerate.end());
    DataMatrix<int64_t> label(F.size(), 1);
    DataMatrix<uint8_t> side(F.size(), 2);
    for (size_t f = 0; f < F.size(); ++f) {
        const bool right = (V[F[f][0]] + V[F[f][1]] + V[F[f][2]]).x() / 3 > 2;
        // The two the cleaning drops have values no other face has.
        label(f, 0) = f < 2 ? 7 : right ? limit : 0;
        side(f, 0) = f < 2 || !right;
        side(f, 1) = f < 2 || right;
    }
    const nlohmann::json options = {
        {"feature_fields", {"patch_id", "side"}},
        {"length_abs", 0.7},
        {"max_iterations", 1}};

    MeshData surface;
    surface.V.resize(V.size(), 3);
    for (size_t i = 0; i < V.size(); ++i) surface.V.row(i) = V[i];
    auto& tris = surface.cells[CellType::Triangle];
    tris.connectivity.resize(F.size(), 3);
    for (size_t i = 0; i < F.size(); ++i) {
        for (int k = 0; k < 3; ++k) tris.connectivity(i, k) = int64_t(F[i][k]);
    }
    tris.attributes["patch_id"].values = label;
    tris.attributes["side"] = {side, {"left", "right"}};
    MeshData with_normal = surface;
    DataMatrix<double> normal = DataMatrix<double>::Zero(F.size(), 9);
    for (int c = 0; c < 3; ++c) normal.col(3 * c + 2).setOnes(); // (0, 0, 1) at every corner
    with_normal.cells[CellType::Triangle].attributes["normal"].values = normal;
    nlohmann::json memory = options;
    memory["input"] = "surface";
    memory["feature_fields"].push_back("normal");
    const auto result = isotropic_remeshing(memory, MeshMap{{"surface", with_normal}}).first;

    const MeshData& mesh = result.at("mesh");
    const auto& block = mesh.cell(CellType::Triangle);
    CHECK(block.attributes.size() == 2); // patch_id and side; not the normal
    REQUIRE(std::holds_alternative<DataMatrix<int64_t>>(block.attributes.at("patch_id").values));
    REQUIRE(std::holds_alternative<DataMatrix<uint8_t>>(block.attributes.at("side").values));
    CHECK(block.attributes.at("patch_id").labels.empty());
    CHECK(block.attributes.at("side").labels == std::vector<std::string>{"left", "right"});
    const auto& patch = std::get<DataMatrix<int64_t>>(block.attributes.at("patch_id").values);
    const auto& sides = std::get<DataMatrix<uint8_t>>(block.attributes.at("side").values);
    // Each triangle has its side's values, and the curves are the edges between the sides.
    std::set<int64_t> seen;
    std::map<Edge, std::set<int64_t>> labels_at;
    for (Eigen::Index f = 0; f < patch.rows(); ++f) {
        const bool right = patch(f, 0) == limit;
        seen.insert(patch(f, 0));
        CHECK(int(sides(f, 0)) == int(!right));
        CHECK(int(sides(f, 1)) == int(right));
        for (int k = 0; k < 3; ++k) {
            const double x = mesh.V(block.connectivity(f, k), 0);
            CHECK((right ? x >= 2 - 1e-9 : x <= 2 + 1e-9));
            const size_t a = block.connectivity(f, k), b = block.connectivity(f, (k + 1) % 3);
            labels_at[{{std::min(a, b), std::max(a, b)}}].insert(patch(f, 0));
        }
    }
    CHECK(seen == std::set<int64_t>{0, limit});
    std::set<Edge> curves;
    const auto& lines = result.at("edges").cell(CellType::Edge).connectivity;
    for (Eigen::Index i = 0; i < lines.rows(); ++i) {
        curves.insert(
            {{size_t(std::min(lines(i, 0), lines(i, 1))),
              size_t(std::max(lines(i, 0), lines(i, 1)))}});
    }
    CHECK(!curves.empty());
    for (const auto& [e, labels] : labels_at) CHECK((labels.size() == 2) == (curves.count(e) > 0));

    // The file route reads the same fields from a .msh file, and writes the same mesh.
    io::write_mesh(dir.path / "in.msh", surface);
    nlohmann::json file = options;
    file["application"] = "isotropic_remeshing";
    file["input"] = "in.msh";
    file["input_dir"] = dir.path.string();
    file["output"] = (dir.path / "out.vtu").string();
    isotropic_remeshing(file);
    const std::string tri = (dir.path / "out.vtu").string();
    CHECK(
        std::vector<double>(mesh.V.data(), mesh.V.data() + mesh.V.size()) ==
        vtu_array<double>(tri, "Points"));
    CHECK(
        std::vector<int64_t>(
            block.connectivity.data(),
            block.connectivity.data() + block.connectivity.size()) ==
        vtu_array<int64_t>(tri, "connectivity"));
    const DataMatrix<double> values = patch.cast<double>();
    CHECK(
        std::vector<double>(values.data(), values.data() + values.size()) ==
        vtu_array<double>(tri, "patch_id"));

    // Refused as soon as the input is read: an Int64 id beyond 2^53, which Float64 does not hold
    // exactly, and a uv or normal that is not per corner, as io::read_mesh reads them.
    const auto refused =
        [&](const std::string& name, const MeshAttribute& field, const char* error) {
            MeshData wrong = surface;
            wrong.cells[CellType::Triangle].attributes[name] = field;
            nlohmann::json j = memory;
            j["feature_fields"] = std::set<std::string>{"patch_id", "side", name};
            CHECK_THROWS_WITH(
                isotropic_remeshing(j, MeshMap{{"surface", wrong}}),
                Catch::Matchers::ContainsSubstring(error));
        };
    DataMatrix<int64_t> big = label;
    big(5, 0) = limit + 1;
    refused(
        "patch_id",
        {big, {}},
        "patch_id in the mesh surface has the value 9007199254740993, beyond 2^53");
    big(5, 0) = -limit - 1;
    refused("patch_id", {big, {}}, "has the value -9007199254740993, beyond 2^53");
    refused(
        "uv",
        {DataMatrix<double>(DataMatrix<double>::Zero(F.size(), 2)), {}},
        "uv in the mesh surface has 2 columns, but uv is per corner, as io::read_mesh reads it: "
        "6 columns");
    refused(
        "normal",
        {DataMatrix<double>(DataMatrix<double>::Zero(F.size(), 3)), {}},
        "normal in the mesh surface has 3 columns, but normal is per corner, as io::read_mesh "
        "reads it: 9 columns");
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

namespace {
/// A closed slab [0, w] x [0, w] x [0, t], outward oriented: n x n squares on the top and the
/// bottom, two triangles each, their inner vertices moved in x and y by up to a fifth of a square
/// (from `seed`), so that the remeshing leaves thin triangles to repair; one row of squares on
/// each side.
void slab(
    const double w,
    const int n,
    const double t,
    const unsigned seed,
    std::vector<Eigen::Vector3d>& V,
    std::vector<std::array<size_t, 3>>& F)
{
    std::mt19937 rng(seed);
    // mt19937's numbers are the same everywhere; uniform_real_distribution's are not.
    const auto jitter = [&rng, w, n]() { return (rng() / 4294967295. * 2 - 1) * 0.2 * w / n; };
    V.clear();
    F.clear();
    const auto sheet = [&](const double z, const bool up) {
        const size_t base = V.size();
        for (int i = 0; i <= n; ++i) {
            for (int j = 0; j <= n; ++j) {
                Eigen::Vector3d p(w * i / n, w * j / n, z);
                if (0 < i && i < n && 0 < j && j < n) {
                    p.x() += jitter();
                    p.y() += jitter();
                }
                V.push_back(p);
            }
        }
        const auto id = [base, n](int i, int j) { return base + size_t(i * (n + 1) + j); };
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                const size_t a = id(i, j), b = id(i + 1, j), c = id(i + 1, j + 1), d = id(i, j + 1);
                if (up) {
                    F.push_back({{a, b, c}});
                    F.push_back({{a, c, d}});
                } else {
                    F.push_back({{a, c, b}});
                    F.push_back({{a, d, c}});
                }
            }
        }
        return id;
    };
    const auto top = sheet(t, true);
    const auto bottom = sheet(0, false);
    // Round the outline, counter-clockwise seen from above.
    std::vector<std::pair<int, int>> loop;
    for (int i = 0; i < n; ++i) loop.emplace_back(i, 0);
    for (int j = 0; j < n; ++j) loop.emplace_back(n, j);
    for (int i = n; i > 0; --i) loop.emplace_back(i, n);
    for (int j = n; j > 0; --j) loop.emplace_back(0, j);
    for (size_t k = 0; k < loop.size(); ++k) {
        const auto [pi, pj] = loop[k];
        const auto [qi, qj] = loop[(k + 1) % loop.size()];
        const size_t a = bottom(pi, pj), b = bottom(qi, qj), c = top(qi, qj), d = top(pi, pj);
        F.push_back({{a, b, c}});
        F.push_back({{a, c, d}});
    }
}

/// The triangles of the slab's top (z = t) that face down, and of its bottom (z = 0) that face
/// up: turned over.
size_t slab_flips(const IsotropicRemeshing& m, const double t)
{
    size_t n = 0;
    for (const auto& f : m.get_faces()) {
        const auto vs = m.oriented_tri_vids(f);
        const auto& p = m.vertex_attrs;
        const double z = (p[vs[1]].pos - p[vs[0]].pos).cross(p[vs[2]].pos - p[vs[0]].pos).z();
        bool top = true, bottom = true;
        for (const size_t v : vs) {
            top = top && p[v].pos.z() > 0.99 * t;
            bottom = bottom && p[v].pos.z() < 0.01 * t;
        }
        if ((top && z <= 0) || (bottom && z >= 0)) ++n;
    }
    return n;
}
} // namespace

TEST_CASE("cleanup-needs-the-angle-before-create-mesh", "[test_remeshing][features]")
{
    // The repairs compare against the input's area vectors, which create_mesh() keeps only
    // after set_cleanup_angle(): without them they would read past the end.
    const CurveOnGrid c;
    IsotropicRemeshing m(c.V, 0);
    m.create_mesh(c.V.size(), c.F);
    CHECK_THROWS(m.cleanup_triangles(30));
    m.set_cleanup_angle(30); // too late
    CHECK_THROWS(m.cleanup_triangles(30));
}

TEST_CASE("output-format-leaves-the-mesh", "[test_remeshing][features]" + release_only)
{
    // A slab labelled by the side each triangle faces, through feature_fields, with a vertex of
    // the top pushed past its neighbour: a fold, where the repairs read the input's orientation,
    // of the same labels only -- next to a thin wall it is noise otherwise. The .vtu output
    // writes the labels and the .obj does not; the mesh is the same.
    const TempDir dir;
    std::vector<Eigen::Vector3d> V;
    std::vector<std::array<size_t, 3>> F;
    slab(4, 8, 0.1, 0, V, F);
    V[1 * 9 + 3].x() += 0.65;
    {
        MshData msh;
        msh.add_face_vertices(V.size(), [&](size_t i) { return V[i]; });
        msh.add_faces(F.size(), [&](size_t i) { return F[i]; });
        msh.add_face_attribute<1>("patch_id", [&](size_t i) {
            const Eigen::Vector3d n = (V[F[i][1]] - V[F[i][0]]).cross(V[F[i][2]] - V[F[i][0]]);
            Eigen::Index axis;
            n.cwiseAbs().maxCoeff(&axis);
            return double(2 * axis + (n[axis] > 0));
        });
        msh.save((dir.path / "in.msh").string());
    }
    std::map<std::string, nlohmann::json> reports;
    for (const std::string ext : {"vtu", "obj"}) {
        isotropic_remeshing(
            {{"application", "isotropic_remeshing"},
             {"input", "in.msh"},
             {"input_dir", dir.path.string()},
             {"output", (dir.path / ("out." + ext)).string()},
             {"report", (dir.path / (ext + ".json")).string()},
             {"feature_fields", {"patch_id"}},
             {"length_abs", 0.6},
             {"eps_rel", 0.001},
             {"max_iterations", 2},
             {"cleanup_angle", 35},
             {"freeze_boundary", false}});
        std::ifstream(dir.path / (ext + ".json")) >> reports[ext];
        reports[ext].erase("time_sec");
    }
    CHECK(reports["vtu"] == reports["obj"]);
    Eigen::MatrixXd OV;
    Eigen::MatrixXi OF;
    REQUIRE(igl::read_triangle_mesh((dir.path / "out.obj").string(), OV, OF));
    const auto conn = vtu_array<int64_t>((dir.path / "out.vtu").string(), "connectivity");
    REQUIRE(conn.size() == size_t(3 * OF.rows()));
    size_t differ = 0;
    for (Eigen::Index f = 0; f < OF.rows(); ++f) {
        for (int k = 0; k < 3; ++k) differ += conn[3 * f + k] != OF(f, k);
    }
    CHECK(differ == 0);
}

TEST_CASE("cleanup-folds-nothing-on-a-thin-wall", "[test_remeshing][features]" + release_only)
{
    // A slab thinner than a triangle is wide: the input next to a triangle of the top holds the
    // bottom too, whose area vectors cancel the top's, so the input's orientation there is
    // noise. The remeshing turns nothing over, and the repairs may not either: they took that
    // orientation for the fold check, and on these slabs let a fold through and left a needle of
    // under 0.1 degrees. With the fold check kept, the worst angle is no worse than without them.
    const double t = 0.1;
    for (const unsigned seed : {1u, 10u}) {
        INFO("seed " << seed);
        std::vector<Eigen::Vector3d> V;
        std::vector<std::array<size_t, 3>> F;
        slab(4, 8, t, seed, V, F);
        const double eps = 0.001 * std::sqrt(4 * 4 * 2 + t * t); // eps_rel 0.001
        std::array<double, 2> angle;
        for (const bool cleanup : {false, true}) {
            INFO("cleanup " << cleanup);
            IsotropicRemeshing m(V, 0);
            if (cleanup) m.set_cleanup_angle(35);
            m.create_mesh(V.size(), F, {}, false, eps);
            m.uniform_remeshing(0.6, 2);
            REQUIRE(m.check_mesh_connectivity_validity());
            CHECK(slab_flips(m, t) == 0);
            angle[cleanup] = smallest_angle(m);
        }
        CHECK(angle[1] >= angle[0] - 1e-9);
    }
}

TEST_CASE("cleanup-collapses-keep-the-length-limit", "[test_remeshing][features]")
{
    // A grid of unit squares with (4, 4) pulled almost onto (5, 4): thin triangles, whose short
    // edge the repairs try to collapse first. Every edge the collapse leaves is about 1, over 4/3
    // of the target 0.6, so once a collapse pass has set that target the repairs refuse it --
    // with limit_collapse_length or without -- and mend the triangles another way. Without a
    // target, they collapse it.
    std::vector<Eigen::Vector3d> V;
    std::vector<std::array<size_t, 3>> F;
    grid(8, V, F);
    V[4 * 9 + 4] = Eigen::Vector3d(4.93, 4, 0);
    for (const auto& [limit, target] :
         {std::pair{false, false}, std::pair{false, true}, std::pair{true, true}}) {
        INFO("limit " << limit << ", target " << target);
        IsotropicRemeshing m(V, 0);
        m.set_cleanup_angle(30);
        m.create_mesh(V.size(), F);
        if (target) {
            // With the limit, the collapse pass refuses the collapse too.
            m.set_limit_collapse_length(true);
            m.collapse_remeshing(0.6);
            REQUIRE(m.get_vertices().size() == V.size());
        }
        m.set_limit_collapse_length(limit);
        REQUIRE(smallest_angle(m) < 5);
        CHECK(m.cleanup_triangles(30) > 0);
        REQUIRE(m.check_mesh_connectivity_validity());
        CHECK(smallest_angle(m) > 20);
        CHECK((m.get_vertices().size() == V.size()) == target);
    }
}

TEST_CASE(
    "features-history-replays-to-the-output",
    "[test_remeshing][features][history]" + release_only)
{
    // The passes this adds edit the mesh through the recorded operations only -- the cleanup's
    // collapses, swaps and moves (its trial positions are put back before the move runs), the
    // collapses turned around for a fixed vertex, the swaps by quality -- so with all of them on,
    // the history still replays to the output, ids and positions exact, and carries the curves
    // to the output's feature edges. The mesh of the cleanup test: thin triangles at the curve
    // and a fold.
    CurveOnGrid c;
    c.V[3 * 9 + 5] = Eigen::Vector3d(3.1, 4.05, 0);
    c.V[6 * 9 + 6] = Eigen::Vector3d(7.3, 6.2, 0);
    for (const int threads : {0, 4}) {
        INFO("threads " << threads);
        IsotropicRemeshing m(c.V, threads);
        m.set_swap_quality(true);
        m.set_limit_collapse_length(true);
        m.set_cleanup_angle(30);
        m.create_mesh(c.V.size(), c.F, {}, true, 0.1);
        m.set_feature_edges(c.edges, c.ids);
        m.set_fixed_vertices(c.fixed);
        m.set_feature_envelope(0.1);
        // As isotropic_remeshing() starts it: the feature edges are the snapshot's features.
        const auto rec = m.start_history([&m](size_t v) { return m.vertex_attrs.at(v).pos; });
        for (const auto& e : m.feature_edges()) rec->history().features.push_back({{e[0], e[1]}});
        // The input's thin and folded triangles first, so that the cleanup has work whatever the
        // passes leave.
        CHECK(m.cleanup_triangles(30) > 0);
        REQUIRE(m.uniform_remeshing(0.6, 2));
        m.consolidate_mesh();
        m.stop_history();
        REQUIRE(m.check_mesh_connectivity_validity());
        check_curve_on_grid(m);

        const history::SurfaceHistory& h = rec->history();
        std::map<history::EditKind, size_t> kinds;
        for (const auto& e : h.edits) ++kinds[e.kind];
        CHECK(kinds[history::EditKind::split_edge] > 0);
        CHECK(kinds[history::EditKind::collapse_edge] > 0);
        CHECK(kinds[history::EditKind::swap_edge] > 0);
        CHECK(kinds[history::EditKind::move_vertices] > 0);

        history::Replay r(h);
        r.apply_all(h);
        const auto out_v = m.get_vertices();
        const auto out_f = m.get_faces();
        REQUIRE(r.vertices().size() == out_v.size());
        REQUIRE(r.faces().size() == out_f.size());
        size_t bad_v = 0, bad_f = 0;
        for (const auto& t : out_v) {
            const size_t v = t.vid(m);
            if (r.vertices()[v] != m.vertex_attrs.at(v).pos) ++bad_v;
        }
        for (const auto& t : out_f) {
            const auto a = m.oriented_tri_vids(t.fid(m));
            const auto& b = r.faces()[t.fid(m)];
            if (a[0] != b[0] || a[1] != b[1] || a[2] != b[2]) ++bad_f;
        }
        CHECK(bad_v == 0);
        CHECK(bad_f == 0);
        std::set<std::array<uint64_t, 2>> features;
        for (const auto& e : m.feature_edges()) features.insert({{e[0], e[1]}});
        CHECK(r.features() == features);
    }
}
