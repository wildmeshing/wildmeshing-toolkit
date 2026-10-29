#include <wmtk/components/tracking/VolumeMap.hpp>
#include <wmtk/components/tracking/volume_geometry.hpp>
#include <wmtk/components/tracking/volume_io.hpp>
#include <wmtk/history/VolumeHistory.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <string>

using namespace wmtk;
using namespace wmtk::components::tracking;

TEST_CASE("volume-history-save-load", "[volume_map]")
{
    history::VolumeHistory h;
    h.regions = {{}, {0}, {0, 3}};
    h.surface_edit_offset = 2;
    history::VolumeEdit e;
    e.op = history::VolumeOp::collapse;
    e.first_edit = 7;
    e.n_edits = 1;
    e.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
    e.surface_before = {0, 1, history::kNoVertex, 2, 3};
    e.surface_after = {history::kNoVertex, 1, history::kNoVertex, 2, 3};
    e.moved = {{1, Eigen::Vector3d(0.5, 0, 0)}};
    e.tets = {{{0, 1, 2, 3}}, {{1, 2, 3, 4}}};
    e.regions = {1, 2};
    e.removed = {0};
    e.added = {{{4, 2, 1, 3}}};
    e.added_regions = {1};
    e.op_vertices = {0, 1};
    h.edits.push_back(e);
    const auto path = (std::filesystem::temp_directory_path() /
                       ("wmtk_volume_history_" + std::to_string(std::random_device{}()) + ".vol"))
                          .string();
    h.save(path);
    const auto g = history::VolumeHistory::load(path);
    std::filesystem::remove(path);
    CHECK(g.regions == h.regions);
    CHECK(g.surface_edit_offset == 2);
    REQUIRE(g.edits.size() == 1);
    const auto& f = g.edits[0];
    CHECK(f.op == e.op);
    CHECK(f.first_edit == 7);
    CHECK(f.positions == e.positions);
    CHECK(f.surface_before == e.surface_before);
    CHECK(f.surface_after == e.surface_after);
    CHECK(f.moved == e.moved);
    CHECK(f.tets == e.tets);
    CHECK(f.regions == e.regions);
    CHECK(f.removed == e.removed);
    CHECK(f.added == e.added);
    CHECK(f.added_regions == e.added_regions);
    CHECK(f.op_vertices == e.op_vertices);
    std::vector<uint32_t> ra;
    const auto after = f.tets_after(&ra);
    CHECK(after.size() == 2);
    CHECK(ra == std::vector<uint32_t>{2, 1});
    CHECK(f.positions_after()[1] == Eigen::Vector3d(0.5, 0, 0));
}

namespace {

std::string temp_path(const std::string& name)
{
    return (std::filesystem::temp_directory_path() /
            ("wmtk_" + name + "_" + std::to_string(std::random_device{}()) + ".vol"))
        .string();
}

/// A volume history file written byte by byte (VolumeHistory.cpp has the layout), to damage.
struct RawVolumeFile
{
    std::vector<uint8_t> bytes;
    void u8(const uint8_t b) { bytes.push_back(b); }
    void varint(uint64_t n)
    {
        do {
            uint8_t b = n & 0x7F;
            n >>= 7;
            if (n) b |= 0x80;
            bytes.push_back(b);
        } while (n);
    }
    void f64(const double x)
    {
        uint8_t b[8];
        std::memcpy(b, &x, 8);
        bytes.insert(bytes.end(), b, b + 8);
    }
    /// The header: no offset, no surface key, the regions {} and {1}, no names, n_edits edits.
    void header(const uint64_t n_edits)
    {
        for (const char c : std::string("WMTKVOLH")) u8(uint8_t(c));
        const uint32_t version = 2;
        uint8_t v[4];
        std::memcpy(v, &version, 4);
        bytes.insert(bytes.end(), v, v + 4);
        varint(0); // surface_edit_offset
        u8(0); // no surface key
        varint(2); // regions
        varint(0);
        varint(1);
        varint(2); // the tag 1, zigzagged
        varint(0); // tag names
        varint(n_edits);
    }
    /// The vertices of an edit: ids from `first` up, each with its position, on no surface.
    void vertices(const std::vector<Eigen::Vector3d>& P, const uint64_t first = 0)
    {
        varint(P.size());
        for (size_t v = 0; v < P.size(); ++v) {
            varint(v == 0 ? 2 * first : 2); // zigzag of the difference from the previous id
            u8(1);
            for (int k = 0; k < 3; ++k) f64(P[v][k]);
        }
    }
    void tet(const std::array<uint32_t, 4>& t, const uint32_t region)
    {
        for (const uint32_t v : t) varint(v);
        varint(region);
    }
    std::string save(const std::string& name) const
    {
        const std::string path = temp_path(name);
        std::ofstream out(path, std::ios::binary);
        out.write(reinterpret_cast<const char*>(bytes.data()), std::streamsize(bytes.size()));
        return path;
    }
};

const std::vector<Eigen::Vector3d> kFive = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};

/// A 2-3 swap of the five points: (0,1,2,3) and (1,2,3,4) become three tets around (0, 4).
RawVolumeFile swap_file()
{
    RawVolumeFile f;
    f.header(1);
    f.u8(uint8_t(history::VolumeOp::swap_23));
    f.varint(3); // first_edit
    f.varint(1); // n_edits
    f.vertices(kFive);
    f.varint(0); // moved
    f.varint(2);
    f.tet({{0, 1, 2, 3}}, 0);
    f.tet({{1, 2, 3, 4}}, 1);
    f.varint(0); // op vertices
    f.u8(0); // not derived
    f.varint(2); // removed
    f.varint(0);
    f.varint(1);
    f.varint(3); // added
    f.tet({{0, 1, 2, 4}}, 0);
    f.tet({{0, 2, 3, 4}}, 0);
    f.tet({{0, 3, 1, 4}}, 1);
    return f;
}

/// Loads a damaged file: it must throw, not crash.
void check_corrupt(const RawVolumeFile& f, const std::string& name)
{
    const std::string path = f.save(name);
    CHECK_THROWS_AS(history::VolumeHistory::load(path), std::runtime_error);
    std::filesystem::remove(path);
}

} // namespace

TEST_CASE("volume-history-damaged", "[volume_map]")
{
    {
        // the file the damaged ones are made from loads
        const std::string path = swap_file().save("swap");
        const auto h = history::VolumeHistory::load(path);
        std::filesystem::remove(path);
        REQUIRE(h.edits.size() == 1);
        CHECK(h.edits[0].removed == std::vector<uint32_t>{0, 1});
        CHECK(h.edits[0].added.size() == 3);
    }
    SECTION("a swap that removes no tet")
    {
        RawVolumeFile f;
        f.header(1);
        f.u8(uint8_t(history::VolumeOp::swap_32));
        f.varint(3);
        f.varint(1);
        f.vertices(kFive);
        f.varint(0);
        f.varint(1);
        f.tet({{0, 1, 2, 3}}, 0);
        f.varint(0);
        f.u8(0);
        f.varint(0); // removed: none
        f.varint(1);
        f.tet({{1, 2, 3, 4}}, 0);
        check_corrupt(f, "swap_none");
    }
    SECTION("a collapse of a vertex into itself")
    {
        RawVolumeFile f;
        f.header(1);
        f.u8(uint8_t(history::VolumeOp::collapse));
        f.varint(3);
        f.varint(1);
        f.vertices(kFive);
        f.varint(0);
        f.varint(2);
        f.tet({{0, 1, 2, 3}}, 0);
        f.tet({{1, 2, 3, 4}}, 0);
        f.varint(2); // op vertices: 1, 1
        f.varint(1);
        f.varint(1);
        f.u8(1); // derived
        check_corrupt(f, "collapse_same");
    }
    SECTION("a tet with a repeated vertex")
    {
        RawVolumeFile f;
        f.header(1);
        f.u8(uint8_t(history::VolumeOp::smooth));
        f.varint(3);
        f.varint(1);
        f.vertices(kFive);
        f.varint(0);
        f.varint(1);
        f.tet({{0, 1, 1, 3}}, 0);
        f.varint(1);
        f.varint(0);
        f.u8(0);
        f.varint(0);
        f.varint(0);
        check_corrupt(f, "repeated");
    }
    SECTION("a vertex id far past those read so far")
    {
        // 2^30 in a file of a few hundred bytes: refused before anything is sized by it
        RawVolumeFile f;
        f.header(1);
        f.u8(uint8_t(history::VolumeOp::smooth));
        f.varint(3);
        f.varint(1);
        f.vertices(kFive, uint64_t(1) << 30);
        f.varint(0);
        f.varint(1);
        f.tet({{0, 1, 2, 3}}, 0);
        f.varint(1);
        f.varint(0);
        f.u8(0);
        f.varint(0);
        f.varint(0);
        check_corrupt(f, "big_id");
    }
    SECTION("a position that is not a number")
    {
        std::vector<Eigen::Vector3d> P = kFive;
        P[2][1] = std::numeric_limits<double>::quiet_NaN();
        RawVolumeFile f;
        f.header(1);
        f.u8(uint8_t(history::VolumeOp::smooth));
        f.varint(3);
        f.varint(1);
        f.vertices(P);
        f.varint(0);
        f.varint(1);
        f.tet({{0, 1, 2, 3}}, 0);
        f.varint(1);
        f.varint(0);
        f.u8(0);
        f.varint(0);
        f.varint(0);
        check_corrupt(f, "nan");
    }
    SECTION("two operations that made the same surface edit")
    {
        RawVolumeFile one = swap_file();
        RawVolumeFile f;
        f.header(2);
        const size_t header = f.bytes.size();
        const std::vector<uint8_t> edit(one.bytes.begin() + header, one.bytes.end());
        f.bytes.insert(f.bytes.end(), edit.begin(), edit.end());
        // the second's vertices are the first's: ids 0..4 again
        f.bytes.insert(f.bytes.end(), edit.begin(), edit.end());
        check_corrupt(f, "same_edit");
    }
    SECTION("every truncation and every bit flipped")
    {
        // Each one loads or throws; none crashes or runs out of memory.
        const RawVolumeFile f = swap_file();
        size_t loaded = 0;
        for (size_t n = 0; n < f.bytes.size(); ++n) {
            RawVolumeFile g;
            g.bytes.assign(f.bytes.begin(), f.bytes.begin() + n);
            const std::string path = g.save("truncated");
            CHECK_THROWS(history::VolumeHistory::load(path));
            std::filesystem::remove(path);
        }
        for (size_t n = 12; n < f.bytes.size(); ++n) {
            for (int bit = 0; bit < 8; ++bit) {
                RawVolumeFile g = f;
                g.bytes[n] ^= uint8_t(1 << bit);
                const std::string path = g.save("flipped");
                try {
                    history::VolumeHistory::load(path);
                    ++loaded;
                } catch (const std::runtime_error&) {
                }
                std::filesystem::remove(path);
            }
        }
        CHECK(loaded > 0); // flips in positions, say
    }
}

TEST_CASE("volume-history-save-checks", "[volume_map]")
{
    // An edit put together by hand is checked as a file's is, and saved with its ids renumbered
    // densely: ids of its own far apart load as 0, 1, ...
    history::VolumeHistory h;
    h.regions = {{}};
    history::VolumeEdit e;
    e.op = history::VolumeOp::smooth;
    e.first_edit = 0;
    e.n_edits = 1;
    e.positions = {kFive[0], kFive[1], kFive[2], kFive[3]};
    e.vertices = {1000, 7, uint64_t(1) << 40, 3};
    e.surface_before = e.surface_after = {0, 1, 2, history::kNoVertex};
    e.moved = {{0, Eigen::Vector3d(0.1, 0.1, 0.1)}};
    e.tets = {{{0, 1, 2, 3}}};
    e.regions = {0};
    e.op_vertices = {0};
    h.edits.push_back(e);
    const std::string path = temp_path("save_checks");
    h.save(path);
    const auto g = history::VolumeHistory::load(path);
    REQUIRE(g.edits.size() == 1);
    CHECK(g.edits[0].vertices == std::vector<uint64_t>{0, 1, 2, 3});
    CHECK(g.edits[0].positions_after()[0] == Eigen::Vector3d(0.1, 0.1, 0.1));

    h.edits[0].surface_after.pop_back();
    CHECK_THROWS_AS(h.save(path), std::runtime_error);
    h.edits[0] = e;
    h.edits[0].regions = {1};
    CHECK_THROWS_AS(h.save(path), std::runtime_error);
    h.edits[0] = e;
    h.edits[0].op_vertices = {};
    CHECK_THROWS_AS(h.save(path), std::runtime_error);
    std::filesystem::remove(path);
}

namespace {

/// A square of two triangles in the plane z = 0, its corners moved one by one in four edits.
history::SurfaceHistory square_history()
{
    history::SurfaceHistory h;
    h.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    h.vertex_alive.assign(4, 1);
    h.faces = {{{0, 1, 2}}, {{0, 2, 3}}};
    h.face_alive.assign(2, 1);
    for (uint64_t v = 0; v < 4; ++v) {
        history::Edit e{history::EditKind::move_vertices, {}, {}, {}};
        e.moves.push_back({v, h.vertices[v]});
        h.edits.push_back(e);
    }
    return h;
}

/// The move of square_history()'s edit 1 as a volume history saw it: the tets on either side of
/// the square at corner 1, a cone over it from (0.5, 0.5, +-1).
history::VolumeHistory square_volume(const history::SurfaceHistory& h)
{
    history::VolumeHistory v;
    v.regions = {{0}, {1}};
    v.surface = history::surface_key(h);
    history::VolumeEdit e;
    e.op = history::VolumeOp::smooth;
    e.first_edit = 1;
    e.n_edits = 1;
    e.positions =
        {h.vertices[0], h.vertices[1], h.vertices[2], h.vertices[3], {0.5, 0.5, 1}, {0.5, 0.5, -1}};
    e.surface_before = {0, 1, 2, 3, history::kNoVertex, history::kNoVertex};
    e.surface_after = e.surface_before;
    e.tets = {{{0, 1, 2, 4}}, {{0, 2, 3, 4}}, {{0, 2, 1, 5}}, {{0, 3, 2, 5}}};
    e.regions = {1, 1, 0, 0};
    e.op_vertices = {1};
    v.edits.push_back(e);
    return v;
}

} // namespace

TEST_CASE("volume-map-pairing", "[volume_map]")
{
    const history::SurfaceHistory h = square_history();
    {
        const VolumeMap map(h, square_volume(h));
        CHECK(map.history().edits.size() == 1);
    }
    // saved and loaded, the key is the same
    {
        const std::string path = temp_path("pairing");
        h.save(path);
        const auto g = history::SurfaceHistory::load(path);
        std::filesystem::remove(path);
        CHECK(history::surface_key(g) == history::surface_key(h));
    }
    // a surface history with another edit, or one more
    history::SurfaceHistory other = h;
    other.edits[2].moves[0].pos[2] = 1e-3;
    CHECK(history::surface_key(other) != history::surface_key(h));
    CHECK_THROWS_AS(VolumeMap(other, square_volume(h)), std::runtime_error);
    other = h;
    other.edits.push_back(other.edits.back());
    CHECK_THROWS_AS(VolumeMap(other, square_volume(h)), std::runtime_error);
    // without a key, what the pair can be checked by alone: the surface edits it names are
    // there, and so are the surface vertices
    history::VolumeHistory v = square_volume(h);
    v.surface.reset();
    v.edits[0].first_edit = 4;
    CHECK_THROWS_AS(VolumeMap(h, v), std::runtime_error);
    v = square_volume(h);
    v.surface.reset();
    v.surface_edit_offset = 3;
    CHECK_THROWS_AS(VolumeMap(h, v), std::runtime_error);
    v = square_volume(h);
    v.surface.reset();
    v.edits[0].surface_before[4] = 9;
    CHECK_THROWS_AS(VolumeMap(h, v), std::runtime_error);
    // and the key goes through a file
    v = square_volume(h);
    const std::string path = temp_path("pairing_volume");
    v.save(path);
    const auto w = history::VolumeHistory::load(path);
    std::filesystem::remove(path);
    REQUIRE(w.surface);
    CHECK(*w.surface == history::surface_key(h));
}

TEST_CASE("volume-map-non-finite", "[volume_map]")
{
    // A point that is not a number is passed by, whatever else is mapped with it: a rational of a
    // NaN or an infinity is a floating point exception in GMP.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const Eigen::Vector3d a(0, 0, 0), b(1, 0, 0), c(0, 1, 0), d(0, 0, 1);
    CHECK_FALSE(volume::exact_barycentric(Eigen::Vector3d(nan, 0, 0), a, b, c, d).allFinite());
    CHECK_FALSE(volume::exact_barycentric(Eigen::Vector3d(0, inf, 0), a, b, c, d).allFinite());
    CHECK_FALSE(volume::exact_barycentric(a, Eigen::Vector3d(nan, 0, 0), b, c, d).allFinite());
    const volume::Tets T = {{{0, 1, 2, 3}}};
    const volume::Positions X = {a, b, c, d};
    const auto any = [](size_t) { return true; };
    CHECK_FALSE(volume::locate_in(T, X, Eigen::Vector3d(nan, 0.1, 0.1), any));
    CHECK(volume::locate_in(T, X, Eigen::Vector3d(0.1, 0.1, 0.1), any));

    const history::SurfaceHistory h = square_history();
    const VolumeMap map(h, square_volume(h));
    std::vector<VolumePoint> pts = {
        {Eigen::Vector3d(0.5, 0.4, 0.3), 1},
        {Eigen::Vector3d(nan, 0.4, 0.3), 1},
        {Eigen::Vector3d(0.5, inf, -inf), 1},
        {},
        {Eigen::Vector3d(0.5, 0.4, -0.3), 0}};
    for (const bool forward : {true, false}) {
        const auto out = forward ? map.to_output(pts) : map.to_input(pts);
        REQUIRE(out.size() == pts.size());
        CHECK(out[0].position.allFinite());
        CHECK(std::isnan(out[1].position[0]));
        CHECK(std::isinf(out[2].position[1]));
        CHECK(std::isnan(out[3].position[2]));
        CHECK(out[4].position.allFinite());
    }

    TetMeshData m;
    m.vertices = {a, b, c, d};
    m.tets = {{{0, 1, 2, 3}}};
    m.groups = {0};
    m.group_sets = {{}};
    const TetLocator locator(m);
    CHECK(locator.locate(Eigen::Vector3d(0.1, 0.1, 0.1)).first == 0);
    CHECK(locator.locate(Eigen::Vector3d(nan, 0.1, 0.1)).first == -1);
    CHECK(locator.locate(Eigen::Vector3d(0.1, inf, 0.1)).first == -1);
    CHECK(locator.locate(Eigen::Vector3d(0.1, 1e300, 0.1)).first == -1);
}

namespace {

/// n^3 cubes of side 1/n, six tets each (Kuhn), every one the right way round, in the group
/// set {} -- or, with `split`, those with x > 1/2 in {"right"}.
TetMeshData kuhn_cube(const int n, const bool split = false)
{
    TetMeshData m;
    const auto id = [n](int x, int y, int z) { return uint64_t((x * (n + 1) + y) * (n + 1) + z); };
    for (int x = 0; x <= n; ++x) {
        for (int y = 0; y <= n; ++y) {
            for (int z = 0; z <= n; ++z) m.vertices.emplace_back(x, y, z);
        }
    }
    for (auto& v : m.vertices) v /= n;
    const int perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
    m.group_sets = split ? std::vector<std::set<std::string>>{{}, {"right"}}
                         : std::vector<std::set<std::string>>{{}};
    for (int x = 0; x < n; ++x) {
        for (int y = 0; y < n; ++y) {
            for (int z = 0; z < n; ++z) {
                for (const auto& p : perms) {
                    int c[3] = {x, y, z};
                    std::array<uint64_t, 4> t;
                    t[0] = id(c[0], c[1], c[2]);
                    for (int k = 0; k < 3; ++k) {
                        ++c[p[k]];
                        t[k + 1] = id(c[0], c[1], c[2]);
                    }
                    const auto& V = m.vertices;
                    if (volume::orient(V[t[0]], V[t[1]], V[t[2]], V[t[3]]) < 0) {
                        std::swap(t[2], t[3]);
                    }
                    m.tets.push_back(t);
                    m.groups.push_back(split && 2 * x >= n ? 1 : 0);
                }
            }
        }
    }
    return m;
}

} // namespace

TEST_CASE("volume-tet-locator", "[volume_map]")
{
    const TetMeshData m = kuhn_cube(5, true);
    const TetLocator locator(m);
    std::mt19937 rng(11);
    const auto u = [&rng]() { return double(rng() % 100000) / 100000; };
    // On the box's faces, and a rounding out of them: every one has a tet, its coordinates
    // putting it back to a rounding.
    size_t found = 0, n = 0, back = 0;
    for (int i = 0; i < 3000; ++i) {
        const int axis = int(rng() % 3), side = int(rng() % 2);
        Eigen::Vector3d p(u(), u(), u());
        p[axis] = side;
        for (const int ulps : {0, 1, 4}) {
            Eigen::Vector3d q = p;
            for (int k = 0; k < ulps; ++k) q[axis] = std::nextafter(q[axis], side ? 2. : -1.);
            const auto [t, b] = locator.locate(q);
            ++n;
            if (t < 0) continue;
            ++found;
            Eigen::Vector3d x = Eigen::Vector3d::Zero();
            for (int k = 0; k < 4; ++k) x += b[k] * m.vertices[m.tets[size_t(t)][k]];
            back += (x - q).norm() < 1e-12;
        }
    }
    CHECK(found == n);
    CHECK(back == n);
    // Farther out, nothing.
    CHECK(locator.locate(Eigen::Vector3d(0.5, 0.5, 1 + 1e-6)).first == -1);
    CHECK(locator.locate(Eigen::Vector3d(-1e-6, 0.5, 0.5)).first == -1);
    // A point on the face between the groups is in both, and found in the one asked for.
    for (int i = 0; i < 200; ++i) {
        const Eigen::Vector3d p(0.6, u(), u());
        for (const uint32_t g : {0u, 1u}) {
            const auto [t, b] = locator.locate(p, g);
            REQUIRE(t >= 0);
            CHECK(m.groups[size_t(t)] == g);
            CHECK(locator.in_group(p, g));
        }
    }
    CHECK_FALSE(locator.in_group(Eigen::Vector3d(0.1, 0.5, 0.5), 1));
    // A mesh with no tets has none to find.
    const TetMeshData empty;
    CHECK(TetLocator(empty).locate(Eigen::Vector3d::Zero()).first == -1);
}

TEST_CASE("volume-chebyshev", "[volume_map]")
{
    // The unit cube's faces: the farthest point from all of them is its centre, at 1/2.
    std::vector<Eigen::Vector3d> N;
    std::vector<double> b;
    for (int k = 0; k < 3; ++k) {
        N.push_back(Eigen::Vector3d::Unit(k));
        b.push_back(1);
        N.push_back(-Eigen::Vector3d::Unit(k));
        b.push_back(0);
    }
    const auto r = volume::chebyshev(N, b, Eigen::Vector3d(5, -3, 2), 10, 10);
    REQUIRE(r);
    CHECK(std::abs(r->second - 0.5) < 1e-12);
    CHECK((r->first - Eigen::Vector3d::Constant(0.5)).norm() < 1e-12);
}

TEST_CASE("volume-spherical-fallback", "[volume_map]")
{
    // A cone from the origin over a band of the sphere around the equator, 300 degrees of it:
    // valid, but no point sees all its faces from inside, so the radial map cannot use it. The
    // spherical fallback keeps every vertex's distance from the apex and lays its direction out
    // again: a valid cone, star-shaped about a point next to the apex.
    using namespace volume;
    const int m = 30;
    Positions P = {Eigen::Vector3d::Zero()};
    for (int k = 0; k <= m; ++k) {
        const double phi = 300. / 180 * M_PI * k / m;
        const double r = 1 + 0.3 * std::sin(3 * phi); // radii vary
        P.push_back(r * Eigen::Vector3d(std::cos(phi), std::sin(phi), -0.2).normalized());
        P.push_back(r * Eigen::Vector3d(std::cos(phi), std::sin(phi), 0.2).normalized());
    }
    Tets T;
    for (int k = 0; k < m; ++k) {
        const Vid a0 = Vid(1 + 2 * k), b0 = a0 + 1, a1 = a0 + 2, b1 = a0 + 3;
        T.push_back({{0, a0, a1, b0}});
        T.push_back({{0, b0, a1, b1}});
    }
    const auto real = StarPatch::make(T, P);
    REQUIRE(real);
    CHECK_FALSE(centre_of(*real));

    const auto Q = spherical(T, P);
    REQUIRE(Q);
    const auto S = StarPatch::make(T, *Q);
    REQUIRE(S);
    const auto c = centre_of(*S);
    REQUIRE(c);
    CHECK(S->sees_all(*c));
    CHECK((*Q)[0] == P[0]); // the apex stays
}
