#include "VolumeHistory.hpp"
#include "BinaryIO.hpp"

#include <algorithm>
#include <cstring>
#include <iterator>
#include <limits>
#include <map>
#include <set>

namespace wmtk::history {

namespace {

// Layout (version 1). Counts and ids are unsigned LEB128 varints, a position three f64, a tag a
// zigzag varint, a surface vertex its id plus one (0: none).
//
//   "WMTKVOLH" u32 version
//   surface_edit_offset
//   n_regions, then per region: n_tags, its tags
//   n_tag_names, then per name: the tag, its length, its bytes
//   n_edits, then per edit, in the order recorded (a reader sorts them by first_edit):
//       u8 op, first_edit, n_edits
//       n_vertices, then per vertex: its id (zigzag, as the difference from the previous
//           vertex's id; the first's from 0), u8 flags, and what the flags say follows --
//           1: its position (else the last one written for the id),
//           2: its surface vertex before (else the last one written for the id, none at first),
//           4: its surface vertex after (else the one before)
//       n_moved, then per moved vertex: local id, position
//       n_tets, then per tet: 4 local ids, region
//       n_op_vertices, their local ids
//       u8 derived: 1 for a collapse whose tets after are its tets before, those with the removed
//           vertex gone and the removed vertex renamed the kept one elsewhere (in place, same
//           region): nothing more; else
//       n_removed, the indices of the tets removed
//       n_added, then per added tet: 4 local ids, region
// A vertex's surface vertex after an edit is what the next edit takes as its last one.
constexpr char kMagic[8] = {'W', 'M', 'T', 'K', 'V', 'O', 'L', 'H'};
constexpr uint32_t kVersion = 1;

using detail::BufferReader;
using detail::BufferWriter;
using detail::unzigzag;
using detail::zigzag;

uint64_t plus_one(const uint64_t v)
{
    return v == kNoVertex ? 0 : v + 1;
}

uint64_t minus_one(const uint64_t v)
{
    return v == 0 ? kNoVertex : v - 1;
}

/// A tet by its vertices, sorted: the same tet whatever its orientation.
std::array<size_t, 4> key_of(std::array<size_t, 4> t)
{
    std::sort(t.begin(), t.end());
    return t;
}

/// What a vertex was last written as.
struct Known
{
    Eigen::Vector3d position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    uint64_t surface = kNoVertex;
    bool written = false;
};

/// A collapse's tets removed and added, from its tets before alone: those holding the removed
/// vertex go; of them, the ones without the kept vertex come back with it in the removed one's
/// place, in the same region.
void derive_collapse(VolumeEdit& e)
{
    const uint32_t i = e.op_vertices[0], j = e.op_vertices[1];
    e.removed.clear();
    e.added.clear();
    e.added_regions.clear();
    for (size_t k = 0; k < e.tets.size(); ++k) {
        const auto& t = e.tets[k];
        if (std::find(t.begin(), t.end(), i) == t.end()) continue;
        e.removed.push_back(uint32_t(k));
        if (std::find(t.begin(), t.end(), j) != t.end()) continue;
        std::array<uint32_t, 4> a = t;
        std::replace(a.begin(), a.end(), i, j);
        e.added.push_back(a);
        e.added_regions.push_back(e.regions[k]);
    }
}

/// Whether e's tets after are what derive_collapse() gives.
bool derivable(const VolumeEdit& e)
{
    if (e.op != VolumeOp::collapse || e.op_vertices.size() != 2) return false;
    VolumeEdit d = e;
    derive_collapse(d);
    if (d.removed != e.removed || d.added_regions.size() != e.added_regions.size()) return false;
    // the same tets, by vertex set, with the same regions
    std::map<std::array<uint32_t, 4>, uint32_t> want, got;
    const auto key = [](std::array<uint32_t, 4> t) {
        std::sort(t.begin(), t.end());
        return t;
    };
    for (size_t k = 0; k < e.added.size(); ++k) want[key(e.added[k])] = e.added_regions[k];
    for (size_t k = 0; k < d.added.size(); ++k) got[key(d.added[k])] = d.added_regions[k];
    return want == got;
}

void encode(BufferWriter& w, const VolumeEdit& e, std::vector<Known>& known)
{
    w.byte(uint8_t(e.op));
    w.varint(e.first_edit);
    w.varint(e.n_edits);
    w.varint(e.positions.size());
    uint64_t previous = 0;
    for (size_t v = 0; v < e.positions.size(); ++v) {
        const uint64_t id = e.vertices[v];
        if (id >= known.size()) known.resize(id + 1);
        Known& k = known[id];
        uint8_t flags = 0;
        if (!k.written || k.position != e.positions[v]) flags |= 1;
        if (e.surface_before[v] != k.surface) flags |= 2;
        if (e.surface_after[v] != e.surface_before[v]) flags |= 4;
        w.varint(zigzag(int64_t(id) - int64_t(previous)));
        previous = id;
        w.byte(flags);
        if (flags & 1) w.position(e.positions[v]);
        if (flags & 2) w.varint(plus_one(e.surface_before[v]));
        if (flags & 4) w.varint(plus_one(e.surface_after[v]));
        k.written = true;
        k.position = e.positions[v];
        k.surface = e.surface_after[v];
    }
    // a moved vertex is last where it moved to
    w.varint(e.moved.size());
    for (const auto& [v, x] : e.moved) {
        w.varint(v);
        w.position(x);
        known[e.vertices[v]].position = x;
    }
    w.varint(e.tets.size());
    for (size_t k = 0; k < e.tets.size(); ++k) {
        for (const uint32_t v : e.tets[k]) w.varint(v);
        w.varint(e.regions[k]);
    }
    w.varint(e.op_vertices.size());
    for (const uint32_t v : e.op_vertices) w.varint(v);
    const bool derived = derivable(e);
    w.byte(derived ? 1 : 0);
    if (derived) return;
    w.varint(e.removed.size());
    for (const uint32_t r : e.removed) w.varint(r);
    w.varint(e.added.size());
    for (size_t k = 0; k < e.added.size(); ++k) {
        for (const uint32_t v : e.added[k]) w.varint(v);
        w.varint(e.added_regions[k]);
    }
}

VolumeEdit decode(BufferReader& r, std::vector<Known>& known, const size_t n_regions)
{
    VolumeEdit e;
    const uint8_t op = r.byte();
    if (op > uint8_t(VolumeOp::smooth)) r.corrupt();
    e.op = VolumeOp(op);
    e.first_edit = r.varint();
    e.n_edits = r.varint();
    const uint64_t nv = r.count(2);
    int64_t previous = 0;
    for (uint64_t v = 0; v < nv; ++v) {
        const int64_t signed_id = previous + unzigzag(r.varint());
        if (signed_id < 0 || signed_id > (int64_t(1) << 40)) r.corrupt();
        const uint64_t id = uint64_t(signed_id);
        previous = signed_id;
        if (id >= known.size()) known.resize(id + 1);
        Known& k = known[id];
        const uint8_t flags = r.byte();
        if (flags & 1) k.position = r.position();
        if (!(flags & 1) && !k.written) r.corrupt();
        const uint64_t before = (flags & 2) ? minus_one(r.varint()) : k.surface;
        const uint64_t after = (flags & 4) ? minus_one(r.varint()) : before;
        e.vertices.push_back(id);
        e.positions.push_back(k.position);
        e.surface_before.push_back(before);
        e.surface_after.push_back(after);
        k.written = true;
        k.surface = after;
    }
    const auto local = [&]() {
        const uint64_t v = r.varint();
        if (v >= nv) r.corrupt();
        return uint32_t(v);
    };
    const auto region = [&]() {
        const uint64_t g = r.varint();
        if (g >= n_regions) r.corrupt();
        return uint32_t(g);
    };
    e.moved.resize(r.count(25));
    for (auto& [v, x] : e.moved) {
        v = local();
        x = r.position();
        known[e.vertices[v]].position = x;
    }
    e.tets.resize(r.count(5));
    e.regions.resize(e.tets.size());
    for (size_t k = 0; k < e.tets.size(); ++k) {
        for (uint32_t& v : e.tets[k]) v = local();
        e.regions[k] = region();
    }
    e.op_vertices.resize(r.count());
    for (uint32_t& v : e.op_vertices) v = local();
    if (r.byte() == 1) {
        if (e.op != VolumeOp::collapse || e.op_vertices.size() != 2) r.corrupt();
        derive_collapse(e);
        return e;
    }
    e.removed.resize(r.count());
    for (uint32_t& t : e.removed) {
        t = uint32_t(r.varint());
        if (t >= e.tets.size()) r.corrupt();
    }
    e.added.resize(r.count(5));
    e.added_regions.resize(e.added.size());
    for (size_t k = 0; k < e.added.size(); ++k) {
        for (uint32_t& v : e.added[k]) v = local();
        e.added_regions[k] = region();
    }
    return e;
}

void write_header(
    detail::Writer& w,
    const uint64_t offset,
    const std::vector<std::vector<int64_t>>& regions,
    const std::map<int64_t, std::string>& tag_names,
    const size_t n_edits)
{
    w.bytes(kMagic, sizeof(kMagic));
    w.put(kVersion);
    w.varint(offset);
    w.varint(regions.size());
    for (const auto& r : regions) {
        w.varint(r.size());
        for (const int64_t t : r) w.varint(zigzag(t));
    }
    w.varint(tag_names.size());
    for (const auto& [t, name] : tag_names) {
        w.varint(zigzag(t));
        w.varint(name.size());
        w.bytes(name.data(), name.size());
    }
    w.varint(n_edits);
}

/// The edits of a buffer, decoded, sorted by surface edit.
std::vector<VolumeEdit> decode_all(
    const uint8_t* p,
    const uint8_t* end,
    const size_t n,
    const size_t n_regions,
    const std::string& path)
{
    BufferReader r(p, end, "volume history", path);
    std::vector<Known> known;
    std::vector<VolumeEdit> edits;
    edits.reserve(n);
    for (size_t k = 0; k < n; ++k) edits.push_back(decode(r, known, n_regions));
    std::stable_sort(edits.begin(), edits.end(), [](const auto& a, const auto& b) {
        return a.first_edit < b.first_edit;
    });
    return edits;
}

} // namespace

std::vector<std::array<uint32_t, 4>> VolumeEdit::tets_after(
    std::vector<uint32_t>* regions_after) const
{
    std::vector<char> gone(tets.size(), 0);
    for (const uint32_t r : removed) gone[r] = 1;
    std::vector<std::array<uint32_t, 4>> out;
    if (regions_after) regions_after->clear();
    for (size_t k = 0; k < tets.size(); ++k) {
        if (gone[k]) continue;
        out.push_back(tets[k]);
        if (regions_after) regions_after->push_back(regions[k]);
    }
    for (size_t k = 0; k < added.size(); ++k) {
        out.push_back(added[k]);
        if (regions_after) regions_after->push_back(added_regions[k]);
    }
    return out;
}

std::vector<Eigen::Vector3d> VolumeEdit::positions_after() const
{
    std::vector<Eigen::Vector3d> p = positions;
    for (const auto& [v, x] : moved) p[v] = x;
    return p;
}

void VolumeHistory::save(const std::string& path) const
{
    // Vertices with no ids of their own are numbered apart, edit by edit.
    std::vector<uint8_t> bytes;
    BufferWriter w(bytes);
    std::vector<Known> known;
    uint64_t next = 0;
    for (const VolumeEdit& e : edits) {
        uint64_t top = next;
        for (const uint64_t id : e.vertices) top = std::max(top, id + 1);
        next = top;
    }
    for (const VolumeEdit& e : edits) {
        if (e.vertices.size() == e.positions.size()) {
            encode(w, e, known);
            continue;
        }
        VolumeEdit f = e;
        f.vertices.clear();
        for (size_t v = 0; v < f.positions.size(); ++v) f.vertices.push_back(next++);
        encode(w, f, known);
    }
    detail::Writer out(path);
    write_header(out, surface_edit_offset, regions, tag_names, edits.size());
    out.bytes(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    out.finish();
}

VolumeHistory VolumeHistory::load(const std::string& path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in) log_and_throw_error("volume history: cannot read {}", path);
    const std::vector<uint8_t> data(
        (std::istreambuf_iterator<char>(in)),
        std::istreambuf_iterator<char>());
    if (data.size() < sizeof(kMagic) + 4 || std::memcmp(data.data(), kMagic, sizeof(kMagic)) != 0) {
        log_and_throw_error("volume history: {} is not a volume history", path);
    }
    BufferReader r(data.data() + sizeof(kMagic), data.data() + data.size(), "volume history", path);
    const uint32_t version = r.get<uint32_t>();
    if (version != kVersion) {
        log_and_throw_error("volume history: {} has version {}, not {}", path, version, kVersion);
    }
    VolumeHistory h;
    h.surface_edit_offset = r.varint();
    h.regions.resize(r.count());
    for (auto& reg : h.regions) {
        reg.resize(r.count());
        for (int64_t& t : reg) t = unzigzag(r.varint());
    }
    for (uint64_t n = r.count(2); n > 0; --n) {
        const int64_t t = unzigzag(r.varint());
        h.tag_names[t] = r.string(r.count());
    }
    const uint64_t n = r.count(4);
    h.edits = decode_all(r.at(), data.data() + data.size(), n, h.regions.size(), path);
    return h;
}

struct VolumeRecorder::Encoder
{
    std::vector<Known> known;
};

VolumeRecorder::VolumeRecorder()
    : m_encoder(std::make_unique<Encoder>())
{}

VolumeRecorder::~VolumeRecorder() = default;

uint32_t VolumeRecorder::region_of(const std::vector<int64_t>& tags)
{
    std::vector<int64_t> t = tags;
    std::sort(t.begin(), t.end());
    t.erase(std::unique(t.begin(), t.end()), t.end());
    const auto it = std::find(m_regions.begin(), m_regions.end(), t);
    if (it != m_regions.end()) return uint32_t(it - m_regions.begin());
    m_regions.push_back(std::move(t));
    return uint32_t(m_regions.size() - 1);
}

void VolumeRecorder::commit(
    const VolumeOp op,
    const std::array<size_t, 2> surface_edits,
    const std::vector<size_t>& op_vertices,
    const Capture& before,
    const Capture& after)
{
    if (surface_edits[1] <= surface_edits[0]) return; // the tracked surface did not change
    VolumeEdit e;
    e.op = op;
    e.first_edit = surface_edits[0];
    e.n_edits = surface_edits[1] - surface_edits[0];
    std::map<size_t, uint32_t> local;
    std::vector<size_t> mesh_vertex;
    for (size_t k = 0; k < before.vertices.size(); ++k) {
        local.emplace(before.vertices[k], uint32_t(e.positions.size()));
        mesh_vertex.push_back(before.vertices[k]);
        e.positions.push_back(before.positions[k]);
        e.surface_before.push_back(before.surface[k]);
        e.surface_after.push_back(kNoVertex);
    }
    for (size_t k = 0; k < after.vertices.size(); ++k) {
        const auto [it, fresh] = local.emplace(after.vertices[k], uint32_t(e.positions.size()));
        if (fresh) { // a vertex only the tets after hold: it was not there before
            mesh_vertex.push_back(after.vertices[k]);
            e.positions.push_back(after.positions[k]);
            e.surface_before.push_back(kNoVertex);
            e.surface_after.push_back(after.surface[k]);
            continue;
        }
        e.surface_after[it->second] = after.surface[k];
        if (after.positions[k] != e.positions[it->second]) {
            e.moved.emplace_back(it->second, after.positions[k]);
        }
    }
    const auto to_local = [&local](const std::array<size_t, 4>& t) {
        return std::array<uint32_t, 4>{
            {local.at(t[0]), local.at(t[1]), local.at(t[2]), local.at(t[3])}};
    };
    std::set<std::array<size_t, 4>> keys_before, keys_after;
    for (const auto& t : before.tets) keys_before.insert(key_of(t));
    for (const auto& t : after.tets) keys_after.insert(key_of(t));
    for (const size_t v : op_vertices) e.op_vertices.push_back(local.at(v));
    std::lock_guard<std::mutex> lock(m_mutex);
    for (const size_t v : mesh_vertex) {
        if (v >= m_id_of.size()) m_id_of.resize(v + 1, kNoVertex);
        if (m_id_of[v] == kNoVertex) m_id_of[v] = m_next_id++;
        e.vertices.push_back(m_id_of[v]);
    }
    for (size_t k = 0; k < before.tets.size(); ++k) {
        if (!keys_after.count(key_of(before.tets[k]))) e.removed.push_back(uint32_t(k));
        e.tets.push_back(to_local(before.tets[k]));
        e.regions.push_back(region_of(before.tags[k]));
    }
    for (size_t k = 0; k < after.tets.size(); ++k) {
        if (keys_before.count(key_of(after.tets[k]))) continue;
        e.added.push_back(to_local(after.tets[k]));
        e.added_regions.push_back(region_of(after.tags[k]));
    }
    BufferWriter w(m_bytes);
    encode(w, e, m_encoder->known);
    ++m_count;
}

void VolumeRecorder::renumber(const std::vector<size_t>& old_to_new)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<uint64_t> next;
    for (size_t v = 0; v < m_id_of.size(); ++v) {
        if (m_id_of[v] == kNoVertex) continue;
        const size_t n = v < old_to_new.size() ? old_to_new[v] : size_t(-1);
        if (n == size_t(-1)) continue;
        if (n >= next.size()) next.resize(n + 1, kNoVertex);
        next[n] = m_id_of[v];
    }
    m_id_of = std::move(next);
}

VolumeHistory VolumeRecorder::history() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    VolumeHistory h;
    h.regions = m_regions;
    h.edits = decode_all(
        m_bytes.data(),
        m_bytes.data() + m_bytes.size(),
        m_count,
        m_regions.size(),
        "the recorder");
    return h;
}

void VolumeRecorder::save(
    const std::string& path,
    const uint64_t surface_edit_offset,
    const std::map<int64_t, std::string>& tag_names) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    detail::Writer out(path);
    write_header(out, surface_edit_offset, m_regions, tag_names, m_count);
    out.bytes(reinterpret_cast<const char*>(m_bytes.data()), m_bytes.size());
    out.finish();
}

size_t VolumeRecorder::size() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_count;
}

size_t VolumeRecorder::bytes() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_bytes.size();
}

} // namespace wmtk::history
