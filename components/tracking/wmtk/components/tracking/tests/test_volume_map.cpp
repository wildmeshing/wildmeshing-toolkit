#include <wmtk/components/tracking/VolumeMap.hpp>
#include <wmtk/components/tracking/tracking.hpp>
#include <wmtk/components/tracking/volume_geometry.hpp>
#include <wmtk/components/tracking/volume_io.hpp>
#include <wmtk/history/VolumeHistory.hpp>
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/utils/io.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <atomic>
#include <thread>

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

TEST_CASE("volume-history-bytes", "[volume_map]")
{
    // A volume history in memory is the bytes of its file, as a surface history is: what the
    // memory route returns, and what read_history and write_history take along with the others.
    using Catch::Matchers::ContainsSubstring;
    history::VolumeHistory h;
    h.regions = {{}, {1}};
    h.tag_names = {{1, "inside"}};
    h.surface = history::SurfaceKey{3, 77};
    history::VolumeEdit e;
    e.op = history::VolumeOp::smooth;
    e.first_edit = 1;
    e.n_edits = 2;
    e.positions = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    e.surface_before = {0, history::kNoVertex, 1, 2};
    e.surface_after = e.surface_before;
    e.moved = {{0, Eigen::Vector3d(0.1, 0.1, 0.1)}};
    e.tets = {{{0, 1, 2, 3}}};
    e.regions = {1};
    e.op_vertices = {0};
    h.edits.push_back(e);

    const std::vector<uint8_t> bytes = h.to_bytes();
    CHECK(std::string(bytes.begin(), bytes.begin() + 8) == "WMTKVOLH");
    CHECK(history::is_volume_history(bytes));
    const auto g = history::VolumeHistory::from_bytes(bytes);
    CHECK(g.regions == h.regions);
    CHECK(g.tag_names == h.tag_names);
    CHECK(g.surface == h.surface);
    REQUIRE(g.edits.size() == 1);
    CHECK(g.edits[0].moved == e.moved);
    CHECK(g.to_bytes() == bytes);

    // The file is the bytes, whichever way it is written.
    const std::string path =
        (std::filesystem::temp_directory_path() /
         ("wmtk_volume_bytes_" + std::to_string(std::random_device{}()) + ".wmtkvol"))
            .string();
    h.save(path);
    CHECK(history::read_history(path) == bytes);
    std::filesystem::remove(path);
    history::write_history(path, bytes);
    CHECK(history::VolumeHistory::load(path).to_bytes() == bytes);
    std::filesystem::remove(path);

    // Each kind where the other is wanted is refused, by name.
    history::SurfaceHistory s;
    s.vertices = {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0)};
    s.vertex_alive.assign(3, 1);
    s.faces = {{{0, 1, 2}}};
    s.face_alive.assign(1, 1);
    CHECK_FALSE(history::is_volume_history(s.to_bytes()));
    CHECK_THROWS_WITH(history::face_size(bytes), ContainsSubstring("is a volume history"));
    CHECK_THROWS_WITH(
        history::SurfaceHistory::from_bytes(bytes),
        ContainsSubstring("is a volume history"));
    CHECK_THROWS_WITH(
        history::VolumeHistory::from_bytes(s.to_bytes()),
        ContainsSubstring("is not a volume history"));
    // Another version, or bytes cut short, are refused before anything is written.
    std::vector<uint8_t> other = bytes;
    other[8] = 9;
    CHECK_THROWS_WITH(history::is_volume_history(other), ContainsSubstring("version 9"));
    CHECK_THROWS_WITH(history::write_history(path, other), ContainsSubstring("version 9"));
    CHECK_THROWS_WITH(
        history::write_history(path, std::vector<uint8_t>(bytes.begin(), bytes.begin() + 10)),
        ContainsSubstring("truncated"));
    CHECK(!std::filesystem::exists(path));
    CHECK_THROWS_WITH(
        history::VolumeHistory::from_bytes(std::vector<uint8_t>(bytes.begin(), bytes.end() - 3)),
        ContainsSubstring("the volume history given"));
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

/// Where square_history() moves corner 1 to.
const Eigen::Vector3d kMovedCorner(0.9, 0.2, 0);

/// A square of two triangles in the plane z = 0, its corners moved one by one in four edits:
/// corner 1 to kMovedCorner, the others where they are.
history::SurfaceHistory square_history()
{
    history::SurfaceHistory h;
    h.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    h.vertex_alive.assign(4, 1);
    h.faces = {{{0, 1, 2}}, {{0, 2, 3}}};
    h.face_alive.assign(2, 1);
    for (uint64_t v = 0; v < 4; ++v) {
        history::Edit e{history::EditKind::move_vertices, {}, {}, {}};
        e.moves.push_back({v, v == 1 ? kMovedCorner : h.vertices[v]});
        h.edits.push_back(e);
    }
    return h;
}

/// The move of square_history()'s edit 1 as a volume history saw it: the tets on either side of
/// the square, a cone over it from (0.5, 0.5, +-1).
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
    e.moved = {{1, kMovedCorner}};
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

namespace {

/// square_volume()'s tets, in the positions before its edit or after, as a mesh file holds them.
MeshData square_mesh(const history::VolumeHistory& v, const bool after)
{
    const history::VolumeEdit& e = v.edits[0];
    const auto P = after ? e.positions_after() : e.positions;
    MeshData m;
    m.V.resize(Eigen::Index(P.size()), 3);
    for (size_t i = 0; i < P.size(); ++i) m.V.row(Eigen::Index(i)) = P[i].transpose();
    auto& T = m.cells[CellType::Tetrahedron].connectivity;
    T.resize(Eigen::Index(e.tets.size()), 4);
    for (size_t t = 0; t < e.tets.size(); ++t) {
        for (int k = 0; k < 4; ++k) T(Eigen::Index(t), k) = e.tets[t][k];
    }
    return m;
}

/// The files of square_history() and square_volume(), and the meshes before and after.
struct SquareFiles
{
    std::string surface = temp_path("square_surface"), volume = temp_path("square_volume");
    std::string in = temp_path("square_in") + ".msh", out = temp_path("square_out") + ".msh";
    SquareFiles()
    {
        const history::SurfaceHistory h = square_history();
        const history::VolumeHistory v = square_volume(h);
        h.save(surface);
        v.save(volume);
        io::write_mesh(in, square_mesh(v, false));
        io::write_mesh(out, square_mesh(v, true));
    }
    ~SquareFiles()
    {
        for (const auto& p : {surface, volume, in, out}) std::filesystem::remove(p);
    }
    VolumeTracking load() const { return VolumeTracking::load(surface, volume, in, out); }
};

/// Points in each of square_volume()'s tets, one of a tet it does not have, one not a number.
std::vector<TetPoint> square_points()
{
    std::vector<TetPoint> pts;
    for (uint64_t t = 0; t < 4; ++t) {
        for (const Eigen::Vector4d& b :
             {Eigen::Vector4d(0.25, 0.25, 0.25, 0.25), Eigen::Vector4d(0.1, 0.6, 0.2, 0.1)}) {
            pts.push_back({t, b});
        }
    }
    pts.push_back({17, Eigen::Vector4d::Constant(0.25)});
    pts.push_back({0, Eigen::Vector4d(std::numeric_limits<double>::quiet_NaN(), 0, 0, 1)});
    return pts;
}

bool same_points(const std::vector<TetPoint>& a, const std::vector<TetPoint>& b)
{
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i].tet != b[i].tet) return false;
        if (a[i].tet != kNoId && (a[i].bary - b[i].bary).norm() > 1e-12) return false;
    }
    return true;
}

} // namespace

TEST_CASE("volume-tracking-moved", "[volume_map]")
{
    // A VolumeTracking moved, copied, or kept in a vector that grows maps as the one loaded:
    // its locators share the meshes, which a move does not leave behind.
    const SquareFiles files;
    VolumeTracking a = files.load();
    const std::vector<TetPoint> pts = square_points();
    const auto there = a.to_output(pts);
    const auto back = a.to_input(there);
    REQUIRE(there.size() == pts.size());
    for (size_t i = 0; i + 2 < pts.size(); ++i) {
        REQUIRE(there[i].tet != kNoId);
        CHECK(back[i].tet != kNoId);
    }
    CHECK(there[pts.size() - 2].tet == kNoId);
    CHECK(there[pts.size() - 1].tet == kNoId);
    const auto maps_as_loaded = [&](const VolumeTracking& t) {
        CHECK(same_points(t.to_output(pts), there));
        CHECK(same_points(t.to_input(there), back));
    };
    VolumeTracking b = std::move(a);
    maps_as_loaded(b);
    VolumeTracking c = b;
    maps_as_loaded(c);
    maps_as_loaded(b);
    std::vector<VolumeTracking> v;
    for (int k = 0; k < 9; ++k) v.push_back(k % 3 == 0 ? files.load() : c);
    v.push_back(std::move(b));
    for (const VolumeTracking& t : v) maps_as_loaded(t);
    a = std::move(v.back());
    v.clear();
    maps_as_loaded(a);
    maps_as_loaded(c);
}

TEST_CASE("volume-tracking-in-memory", "[volume_map]")
{
    // The histories and the meshes as objects map as the files they are saved to.
    const SquareFiles files;
    const VolumeTracking loaded = files.load();
    const history::SurfaceHistory h = square_history();
    const history::VolumeHistory v = square_volume(h);
    const VolumeTracking tracking(h, v, square_mesh(v, false), square_mesh(v, true));
    REQUIRE(tracking.has_meshes());
    CHECK(tracking.input_mesh().tets == loaded.input_mesh().tets);
    const std::vector<TetPoint> pts = square_points();
    const auto there = loaded.to_output(pts);
    CHECK(same_points(tracking.to_output(pts), there));
    CHECK(same_points(tracking.to_input(there), loaded.to_input(there)));

    // Positions, with or without the meshes: as the map gives them, regions and all, and back.
    std::vector<Eigen::Vector3d> xyz;
    std::vector<VolumePoint> want;
    for (const TetPoint& p : pts) {
        if (p.tet >= v.edits[0].tets.size() || !p.bary.allFinite()) continue;
        VolumePoint q;
        q.position.setZero();
        for (int k = 0; k < 4; ++k) {
            q.position += p.bary[k] * v.edits[0].positions[v.edits[0].tets[p.tet][k]];
        }
        xyz.push_back(q.position);
        want.push_back(q);
    }
    xyz.push_back(Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
    want.push_back({});
    want = tracking.map().to_output(want);
    const VolumeTracking positions_only(h, v);
    CHECK_FALSE(positions_only.has_meshes());
    CHECK_THROWS_AS(positions_only.to_output(pts), std::runtime_error);
    CHECK_THROWS_AS(positions_only.input_mesh(), std::runtime_error);
    for (const VolumeTracking* t : {&tracking, &positions_only}) {
        const auto out = t->map_positions(xyz, true);
        REQUIRE(out.size() == xyz.size());
        std::vector<Eigen::Vector3d> there_xyz;
        std::vector<uint32_t> regions;
        for (size_t i = 0; i + 1 < out.size(); ++i) {
            CHECK(out[i].position == want[i].position);
            CHECK(out[i].region == want[i].region);
            CHECK(out[i].region != VolumePoint::kAnyRegion);
            there_xyz.push_back(out[i].position);
            regions.push_back(out[i].region);
        }
        CHECK(std::isnan(out.back().position[0]));
        const auto back = t->map_positions(there_xyz, false, regions);
        for (size_t i = 0; i < back.size(); ++i) {
            CHECK((back[i].position - xyz[i]).norm() < 1e-12);
            CHECK(back[i].region == regions[i]);
        }
        CHECK_THROWS_AS(t->map_positions(xyz, true, {0}), std::runtime_error);
    }

    // A mesh without tets is refused.
    MeshData no_tets = square_mesh(v, false);
    no_tets.cells.clear();
    CHECK_THROWS_AS(VolumeTracking(h, v, no_tets, square_mesh(v, true)), std::runtime_error);
}

TEST_CASE("volume-map-app", "[volume_map]")
{
    // The tracking app's map_volume, on tet points and on positions, writes what VolumeTracking
    // gives; volume reports on the map.
    const SquareFiles files;
    const VolumeTracking loaded = files.load();
    std::vector<TetPoint> pts = square_points();
    pts.pop_back(); // not a number, which a file of points has no way to say
    const std::string points = temp_path("points") + ".txt", out = temp_path("mapped") + ".txt";
    {
        std::ofstream f(points);
        f.precision(17);
        for (const TetPoint& p : pts) {
            f << (p.tet == 17 ? -1 : int64_t(p.tet));
            for (int k = 0; k < 4; ++k) f << " " << p.bary[k];
            f << "\n";
        }
    }
    const nlohmann::json base = {
        {"application", "tracking"},
        {"history", files.surface},
        {"volume_history", files.volume},
        {"tet_mesh", files.in},
        {"output_tet_mesh", files.out},
        {"points", points},
        {"output_points", out}};
    nlohmann::json j = base;
    j["operation"] = "map_volume";
    components::tracking::tracking(j);
    const auto there = loaded.to_output(pts);
    {
        std::ifstream f(out);
        for (size_t i = 0; i < pts.size(); ++i) {
            int64_t t;
            Eigen::Vector4d b;
            REQUIRE(f >> t);
            for (int k = 0; k < 4; ++k) f >> b[k];
            if (there[i].tet == kNoId) {
                CHECK(t == -1);
                continue;
            }
            CHECK(uint64_t(t) == there[i].tet);
            CHECK((b - there[i].bary).norm() < 1e-12);
        }
    }
    {
        std::ofstream f(points);
        f << "0.5 0.4 0.3\n0.5 0.4 -0.3 # a comment\n";
    }
    j["direction"] = "to_input";
    components::tracking::tracking(j);
    {
        std::ifstream f(out);
        Eigen::Vector3d x;
        REQUIRE(f >> x[0] >> x[1] >> x[2]);
        CHECK(x.allFinite());
    }
    j = base;
    j["operation"] = "volume";
    j["samples"] = 50;
    CHECK_NOTHROW(components::tracking::tracking(j));
    for (const auto& p : {points, out}) std::filesystem::remove(p);
}

TEST_CASE("volume-region-labels", "[volume_map]")
{
    // A tet's groups are the labels io::collect_tags gives the tags: two named alike become
    // "a" and "a_", and the untagged tets are "ambient" -- unless a tag is, and then in none.
    const history::SurfaceHistory h = square_history();
    history::VolumeHistory v = square_volume(h);
    v.regions = {{}, {0}, {1}, {2}};
    v.tag_names = {{0, "a"}, {1, "a_"}, {2, "tag_2"}};
    {
        const VolumeMap map(h, v);
        CHECK(map.region_of_groups({"a"}) == 1u);
        CHECK(map.region_of_groups({"a_"}) == 2u);
        CHECK(map.region_of_groups({"ambient"}) == 0u);
        CHECK(map.region_of_groups({}) == 0u);
        CHECK_FALSE(map.region_of_groups({"b"}));
    }
    v.tag_names = {{0, "ambient"}, {1, "ambient_"}};
    {
        const VolumeMap map(h, v);
        CHECK(map.region_of_groups({"ambient"}) == 1u);
        CHECK(map.region_of_groups({"ambient_"}) == 2u);
        CHECK(map.region_of_groups({}) == 0u);
    }

    // The tets above the square tagged with the tag labelled "ambient": a point there keeps its
    // region, follows the corner that moves, and lands in a tet of the tag.
    v = square_volume(h);
    v.regions = {{}, {0}};
    v.tag_names = {{0, "ambient"}};
    std::array<MeshData, 2> meshes = {square_mesh(v, false), square_mesh(v, true)};
    for (MeshData& m : meshes) {
        DataMatrix<uint8_t> tags = DataMatrix<uint8_t>::Zero(4, 1);
        for (int t = 0; t < 4; ++t) tags(t, 0) = v.edits[0].regions[size_t(t)] == 1;
        m.cells[CellType::Tetrahedron].attributes["tags"] = {tags, {"ambient"}};
    }
    const VolumeTracking tracking(h, v, meshes[0], meshes[1]);
    const TetPoint p{0, Eigen::Vector4d(0.1, 0.6, 0.2, 0.1)}; // near corner 1, above
    const auto there = tracking.to_output({p});
    REQUIRE(there[0].tet != kNoId);
    const auto& out = tracking.output_mesh();
    CHECK(out.group_sets[out.groups[there[0].tet]] == std::set<std::string>{"ambient"});
    const auto position = [](const TetMeshData& m, const TetPoint& q) {
        Eigen::Vector3d x = Eigen::Vector3d::Zero();
        for (int k = 0; k < 4; ++k) x += q.bary[k] * m.vertices[m.tets[q.tet][k]];
        return x;
    };
    CHECK((position(out, there[0]) - position(tracking.input_mesh(), p)).norm() > 1e-3);
    CHECK(tracking.map().stats().region_changed == 0);
}

TEST_CASE("volume-map-threads", "[volume_map]")
{
    // One map, queried from several threads at once: each gets what one thread alone gets, and
    // the stats count every point once. The second operation is a collapse whose tets do not
    // fit it, so every point goes through a map that fails, which the stats count.
    const history::SurfaceHistory h = square_history();
    history::VolumeHistory v = square_volume(h);
    history::VolumeEdit bad = v.edits[0];
    bad.op = history::VolumeOp::collapse;
    bad.first_edit = 2;
    bad.positions = bad.positions_after();
    bad.moved.clear();
    bad.op_vertices = {0, 1};
    bad.removed = {0};
    v.edits.push_back(bad);
    const VolumeMap map(h, v);
    std::vector<VolumePoint> pts;
    std::mt19937 rng(3);
    for (int i = 0; i < 1000; ++i) {
        const auto& t = v.edits[0].tets[rng() % 4];
        Eigen::Vector4d b;
        for (int k = 0; k < 4; ++k) b[k] = 1 + double(rng() % 100);
        b /= b.sum();
        VolumePoint p;
        p.position.setZero();
        for (int k = 0; k < 4; ++k) p.position += b[k] * v.edits[0].positions[t[k]];
        pts.push_back(p);
    }
    const auto serial = map.to_output(pts);
    const size_t forward = map.stats().through_failed;
    REQUIRE(forward == pts.size());
    map.to_input(serial);
    const size_t backward = map.stats().through_failed - forward; // not those the collapse took
    REQUIRE(backward > 0);
    const int n_threads = 8, calls = 5;
    std::vector<std::thread> threads;
    std::atomic<size_t> different{0};
    for (int t = 0; t < n_threads; ++t) {
        threads.emplace_back([&]() {
            for (int c = 0; c < calls; ++c) {
                const auto there = map.to_output(pts);
                const auto back = map.to_input(there);
                for (size_t i = 0; i < pts.size(); ++i) {
                    different += there[i].position != serial[i].position ||
                                 there[i].region != serial[i].region;
                }
                map.build_all();
                (void)map.stats();
            }
        });
    }
    for (auto& t : threads) t.join();
    CHECK(different == 0);
    const VolumeMapStats st = map.stats();
    CHECK(st.built == 1);
    CHECK(st.failed == 1);
    REQUIRE(st.failures.size() == 1);
    CHECK(st.failures.begin()->second == 1);
    CHECK(st.through_failed == (forward + backward) * (1 + n_threads * calls));
}

TEST_CASE("volume-map-cache", "[volume_map]")
{
    // The maps built are kept for later calls up to the cache's bytes, the least recently used
    // dropped past them, and a map maps the same whatever it keeps.
    const history::SurfaceHistory h = square_history();
    const history::VolumeHistory v = square_volume(h);
    std::vector<VolumePoint> pts;
    for (const auto& t : v.edits[0].tets) {
        VolumePoint p;
        p.position.setZero();
        for (int k = 0; k < 4; ++k) p.position += 0.25 * v.edits[0].positions[t[k]];
        pts.push_back(p);
    }
    const VolumeMap kept(h, v);
    const auto want = kept.to_output(pts);
    CHECK(kept.stats().kept == 1);
    CHECK(kept.stats().kept_bytes > 0);
    for (const size_t bytes : {size_t(0), size_t(1), kept.stats().kept_bytes}) {
        VolumeMapOptions o;
        o.cache_bytes = bytes;
        const VolumeMap map(h, v, o);
        for (int call = 0; call < 3; ++call) {
            const auto there = map.to_output(pts);
            for (size_t i = 0; i < pts.size(); ++i) CHECK(there[i].position == want[i].position);
        }
        CHECK(map.stats().built == 1);
        CHECK(map.stats().kept == (bytes >= kept.stats().kept_bytes ? 1 : 0));
    }
    CHECK(kept.op_kinds(0) == std::vector<std::string>{"move", "move"});

    // Points far from the operations, however far, are in none, and leave the others' results
    // as they are.
    std::vector<VolumePoint> far = pts;
    far.push_back({Eigen::Vector3d(1e300, 0, 0), 0});
    far.push_back({Eigen::Vector3d(-1e308, 1e308, 0), 1});
    far.push_back({Eigen::Vector3d(0.5, 0.5, 1e9), 1});
    const auto there = kept.to_output(far);
    for (size_t i = 0; i < pts.size(); ++i) CHECK(there[i].position == want[i].position);
    for (size_t i = pts.size(); i < far.size(); ++i) CHECK(there[i].position == far[i].position);
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
    const TetLocator locator(std::make_shared<const TetMeshData>(m));
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
    const TetLocator locator(std::make_shared<const TetMeshData>(m));
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
    CHECK(
        TetLocator(std::make_shared<const TetMeshData>()).locate(Eigen::Vector3d::Zero()).first ==
        -1);
}

TEST_CASE("volume-read-tet-msh", "[volume_map]")
{
    const TetMeshData cube = kuhn_cube(2);
    MeshData m;
    m.V.resize(Eigen::Index(cube.vertices.size()), 3);
    for (size_t v = 0; v < cube.vertices.size(); ++v) {
        m.V.row(Eigen::Index(v)) = cube.vertices[v].transpose();
    }
    auto& T = m.cells[CellType::Tetrahedron].connectivity;
    T.resize(Eigen::Index(cube.tets.size()) + 1, 4);
    for (size_t t = 0; t < cube.tets.size(); ++t) {
        for (int k = 0; k < 4; ++k) T(Eigen::Index(t), k) = int64_t(cube.tets[t][k]);
    }
    T.row(T.rows() - 1) = T.row(3); // a tet twice: two rows, two tets
    SECTION("io::write_mesh, groups in tags columns (simwild)")
    {
        // two tags, the untagged tets "ambient", as io::collect_tags labels them
        DataMatrix<uint8_t> tags = DataMatrix<uint8_t>::Zero(T.rows(), 3);
        for (Eigen::Index r = 0; r < T.rows(); ++r) {
            tags(r, r % 3) = 1;
            if (r % 5 == 0) tags(r, 1) = 1;
        }
        m.cells[CellType::Tetrahedron].attributes["tags"] = {tags, {"tag_0", "tag_1", "ambient"}};
        const std::string path = temp_path("write_mesh") + ".msh";
        io::write_mesh(path, m);
        const TetMeshData d = read_tet_msh(path);
        const MeshData r = io::read_mesh(path);
        std::filesystem::remove(path);
        // the rows io::read_mesh gives, in order, each with its groups
        const auto& R = r.cells.at(CellType::Tetrahedron).connectivity;
        REQUIRE(d.tets.size() == size_t(R.rows()));
        REQUIRE(d.tets.size() == size_t(T.rows()));
        for (size_t t = 0; t < d.tets.size(); ++t) {
            for (int k = 0; k < 4; ++k) CHECK(d.tets[t][k] == uint64_t(T(Eigen::Index(t), k)));
            std::set<std::string> want;
            const char* names[3] = {"tag_0", "tag_1", "ambient"};
            for (int j = 0; j < 3; ++j) {
                if (tags(Eigen::Index(t), j)) want.insert(names[j]);
            }
            CHECK(d.group_sets[d.groups[t]] == want);
        }
        CHECK(tet_mesh_data(m).tets == d.tets);
    }
    SECTION("MshData, no groups (tetwild)")
    {
        wmtk::MshData msh;
        msh.add_tet_vertices(cube.vertices.size(), [&](size_t k) { return cube.vertices[k]; });
        msh.add_tets(cube.tets.size(), [&](size_t k) {
            std::array<size_t, 4> t;
            for (int c = 0; c < 4; ++c) t[c] = size_t(cube.tets[k][c]);
            return t;
        });
        msh.add_tet_attribute<1>("t energy", [](size_t) { return 1.; });
        const std::string path = temp_path("mshdata") + ".msh";
        msh.save(path, true);
        const TetMeshData d = read_tet_msh(path);
        std::filesystem::remove(path);
        CHECK(d.tets == cube.tets);
        CHECK(d.vertices == cube.vertices);
        REQUIRE(d.group_sets.size() == 1);
        CHECK(d.group_sets[0].empty());
    }
    SECTION("a tet once per group (the older layout)")
    {
        // io::read_mesh makes each tet one row with all its groups, and so does read_tet_msh
        const std::string path = std::string(WMTK_DATA_DIR) + "/models/double_sphere.msh";
        const TetMeshData d = read_tet_msh(path);
        const MeshData r = io::read_mesh(path);
        CHECK(d.tets.size() == size_t(r.cells.at(CellType::Tetrahedron).connectivity.rows()));
        std::set<std::array<uint64_t, 4>> distinct;
        for (auto t : d.tets) {
            std::sort(t.begin(), t.end());
            distinct.insert(t);
        }
        CHECK(distinct.size() == d.tets.size());
        CHECK(d.group_sets.size() > 1);
    }
}

namespace {

/// The tets of a collapse of mesh vertex 10 into 11, as TetMesh captures them: the stars of
/// both before, of 11 after. Vertices 10..15 at the five points and one more; 12 is on the
/// surface as surface vertex 7. `base` is added to the mesh ids.
std::pair<history::VolumeCapture, history::VolumeCapture> collapse_captures(const size_t base = 0)
{
    const auto id = [base](size_t v) { return v + base; };
    history::VolumeCapture b, a;
    b.tets = {
        {{id(10), id(11), id(12), id(13)}},
        {{id(10), id(12), id(14), id(13)}},
        {{id(11), id(12), id(13), id(15)}}};
    b.tags = {{1}, {1, 3}, {}};
    b.vertices = {id(10), id(11), id(12), id(13), id(14), id(15)};
    b.positions = {kFive[0], kFive[1], kFive[2], kFive[3], kFive[4], {2, 2, 2}};
    b.surface = {
        history::kNoVertex,
        history::kNoVertex,
        7,
        history::kNoVertex,
        history::kNoVertex,
        history::kNoVertex};
    a.tets = {{{id(11), id(12), id(14), id(13)}}, {{id(11), id(12), id(13), id(15)}}};
    a.tags = {{3, 1}, {}};
    a.vertices = {id(11), id(12), id(13), id(14), id(15)};
    a.positions = {kFive[1], kFive[2], kFive[3], kFive[4], {2, 2, 2}};
    a.surface = {history::kNoVertex, 8, history::kNoVertex, history::kNoVertex, history::kNoVertex};
    return {b, a};
}

} // namespace

TEST_CASE("volume-recorder", "[volume_map]")
{
    const auto [before, after] = collapse_captures();
    SECTION("a commit, its tets after derived")
    {
        history::VolumeRecorder r;
        r.commit(history::VolumeOp::collapse, {{3, 5}}, {10, 11}, before, after);
        r.commit(history::VolumeOp::collapse, {{6, 6}}, {10, 11}, before, after); // no edit
        REQUIRE(r.size() == 1);
        const history::VolumeHistory h = r.history();
        REQUIRE(h.edits.size() == 1);
        const history::VolumeEdit& e = h.edits[0];
        CHECK(e.first_edit == 3);
        CHECK(e.n_edits == 2);
        CHECK(e.positions == before.positions);
        CHECK(e.surface_before == before.surface);
        CHECK(e.surface_after[2] == 8);
        CHECK(e.surface_after[0] == history::kNoVertex); // gone
        CHECK(e.op_vertices == std::vector<uint32_t>{0, 1});
        // regions by tag set, in the order they come; the tets after: the star of 11
        CHECK(h.regions == std::vector<std::vector<int64_t>>{{1}, {1, 3}, {}});
        CHECK(e.regions == std::vector<uint32_t>{0, 1, 2});
        CHECK(e.removed == std::vector<uint32_t>{0, 1});
        std::vector<uint32_t> ra;
        const auto ta = e.tets_after(&ra);
        REQUIRE(ta.size() == 2);
        CHECK(ra == std::vector<uint32_t>{2, 1});

        // The same tets as a swap are not derivable: removed and added are written out, and
        // read back the same.
        history::VolumeRecorder s;
        s.commit(history::VolumeOp::swap_32, {{3, 5}}, {}, before, after);
        CHECK(s.bytes() > r.bytes());
        std::vector<uint32_t> sa;
        CHECK(s.history().edits[0].tets_after(&sa) == ta);
        CHECK(sa == ra);
    }
    SECTION("ids that last through a renumbering")
    {
        history::VolumeRecorder r;
        r.commit(history::VolumeOp::collapse, {{0, 1}}, {10, 11}, before, after);
        const size_t one = r.bytes();
        // The mesh renumbers: v -> v + 100. The same vertices, where they were: the second edit
        // has the first's ids, and writes no position again.
        std::vector<size_t> old_to_new(16, size_t(-1));
        for (size_t v = 10; v < 16; ++v) old_to_new[v] = v + 100;
        r.renumber(old_to_new);
        const auto [b2, a2] = collapse_captures(100);
        r.commit(history::VolumeOp::collapse, {{1, 2}}, {110, 111}, b2, a2);
        const auto h = r.history();
        REQUIRE(h.edits.size() == 2);
        CHECK(h.edits[1].vertices == h.edits[0].vertices);
        CHECK(r.bytes() - one <= one - 6 * 24); // six positions of 24 bytes fewer
        // a vertex the renumbering drops is a new one when it comes back
        std::vector<size_t> drop(116, size_t(-1));
        for (size_t v = 111; v < 116; ++v) drop[v] = v - 100;
        r.renumber(drop);
        const auto [b3, a3] = collapse_captures();
        r.commit(history::VolumeOp::collapse, {{2, 3}}, {10, 11}, b3, a3);
        const auto g = r.history();
        CHECK(g.edits[2].vertices[0] != g.edits[0].vertices[0]);
        CHECK(g.edits[2].vertices[1] == g.edits[0].vertices[1]);
    }
    SECTION("commits from several threads")
    {
        // Each thread its own vertices and surface edits; read back, the edits are in surface
        // edit order, each as committed.
        history::VolumeRecorder r;
        const int n_threads = 8, per_thread = 50;
        std::vector<std::thread> threads;
        for (int t = 0; t < n_threads; ++t) {
            threads.emplace_back([&r, t]() {
                for (int i = 0; i < per_thread; ++i) {
                    const size_t first = size_t(i * n_threads + t);
                    auto [b, a] = collapse_captures(size_t(first) * 16);
                    for (auto& x : b.positions) x[0] += double(first);
                    for (auto& x : a.positions) x[0] += double(first);
                    r.commit(
                        history::VolumeOp::collapse,
                        {{first, first + 1}},
                        {first * 16 + 10, first * 16 + 11},
                        b,
                        a);
                }
            });
        }
        for (auto& t : threads) t.join();
        const auto h = r.history();
        REQUIRE(h.edits.size() == size_t(n_threads * per_thread));
        for (size_t k = 0; k < h.edits.size(); ++k) {
            CHECK(h.edits[k].first_edit == k);
            CHECK(h.edits[k].positions[5] == Eigen::Vector3d(2. + double(k), 2, 2));
            CHECK(h.edits[k].removed.size() == 2);
        }
        CHECK(h.regions.size() == 3);
    }
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
