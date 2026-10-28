#include "VolumeHistory.hpp"
#include "BinaryIO.hpp"
#include "History.hpp"

#include <algorithm>
#include <cstring>
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
//   n_edits, then per edit:
//       u8 op, first_edit, n_edits
//       n_vertices, then per vertex: position, surface vertex before, after
//       n_moved, then per moved vertex: local id, position
//       n_tets, then per tet: 4 local ids, region
//       n_removed, the indices of the tets removed
//       n_added, then per added tet: 4 local ids, region
//       n_op_vertices, their local ids
constexpr char kMagic[8] = {'W', 'M', 'T', 'K', 'V', 'O', 'L', 'H'};
constexpr uint32_t kVersion = 1;

uint64_t zigzag(const int64_t x)
{
    return (uint64_t(x) << 1) ^ uint64_t(x >> 63);
}

int64_t unzigzag(const uint64_t x)
{
    return int64_t(x >> 1) ^ -int64_t(x & 1);
}

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
    detail::Writer w(path);
    w.bytes(kMagic, sizeof(kMagic));
    w.put(kVersion);
    w.varint(surface_edit_offset);
    w.varint(regions.size());
    for (const auto& r : regions) {
        w.varint(r.size());
        for (const int64_t t : r) w.varint(zigzag(t));
    }
    w.varint(edits.size());
    for (const VolumeEdit& e : edits) {
        w.put(uint8_t(e.op));
        w.varint(e.first_edit);
        w.varint(e.n_edits);
        w.varint(e.positions.size());
        for (size_t v = 0; v < e.positions.size(); ++v) {
            w.position(e.positions[v]);
            w.varint(plus_one(e.surface_before[v]));
            w.varint(plus_one(e.surface_after[v]));
        }
        w.varint(e.moved.size());
        for (const auto& [v, x] : e.moved) {
            w.varint(v);
            w.position(x);
        }
        w.varint(e.tets.size());
        for (size_t k = 0; k < e.tets.size(); ++k) {
            for (const uint32_t v : e.tets[k]) w.varint(v);
            w.varint(e.regions[k]);
        }
        w.varint(e.removed.size());
        for (const uint32_t r : e.removed) w.varint(r);
        w.varint(e.added.size());
        for (size_t k = 0; k < e.added.size(); ++k) {
            for (const uint32_t v : e.added[k]) w.varint(v);
            w.varint(e.added_regions[k]);
        }
        w.varint(e.op_vertices.size());
        for (const uint32_t v : e.op_vertices) w.varint(v);
    }
    w.finish();
}

VolumeHistory VolumeHistory::load(const std::string& path)
{
    detail::Reader r(path);
    char magic[sizeof(kMagic)];
    r.bytes(magic, sizeof(magic));
    if (std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
        log_and_throw_error("volume history: {} is not a volume history", path);
    }
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
    h.edits.resize(r.count(4));
    for (VolumeEdit& e : h.edits) {
        const uint8_t op = r.get<uint8_t>();
        if (op > uint8_t(VolumeOp::smooth)) r.corrupt();
        e.op = VolumeOp(op);
        e.first_edit = r.varint();
        e.n_edits = r.varint();
        const uint64_t nv = r.count(26);
        e.positions.resize(nv);
        e.surface_before.resize(nv);
        e.surface_after.resize(nv);
        for (uint64_t v = 0; v < nv; ++v) {
            e.positions[v] = r.position();
            e.surface_before[v] = minus_one(r.varint());
            e.surface_after[v] = minus_one(r.varint());
        }
        const auto local = [&]() {
            const uint64_t v = r.varint();
            if (v >= nv) r.corrupt();
            return uint32_t(v);
        };
        const auto region = [&]() {
            const uint64_t g = r.varint();
            if (g >= h.regions.size()) r.corrupt();
            return uint32_t(g);
        };
        e.moved.resize(r.count(25));
        for (auto& [v, x] : e.moved) {
            v = local();
            x = r.position();
        }
        e.tets.resize(r.count(5));
        e.regions.resize(e.tets.size());
        for (size_t k = 0; k < e.tets.size(); ++k) {
            for (uint32_t& v : e.tets[k]) v = local();
            e.regions[k] = region();
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
        e.op_vertices.resize(r.count());
        for (uint32_t& v : e.op_vertices) v = local();
    }
    return h;
}

uint32_t VolumeRecorder::region_of(const std::vector<int64_t>& tags)
{
    std::vector<int64_t> t = tags;
    std::sort(t.begin(), t.end());
    t.erase(std::unique(t.begin(), t.end()), t.end());
    const auto it = std::find(m_history.regions.begin(), m_history.regions.end(), t);
    if (it != m_history.regions.end()) return uint32_t(it - m_history.regions.begin());
    m_history.regions.push_back(std::move(t));
    return uint32_t(m_history.regions.size() - 1);
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
    for (size_t k = 0; k < before.vertices.size(); ++k) {
        local.emplace(before.vertices[k], uint32_t(e.positions.size()));
        e.positions.push_back(before.positions[k]);
        e.surface_before.push_back(before.surface[k]);
        e.surface_after.push_back(kNoVertex);
    }
    for (size_t k = 0; k < after.vertices.size(); ++k) {
        const auto [it, fresh] = local.emplace(after.vertices[k], uint32_t(e.positions.size()));
        if (fresh) { // a vertex only the tets after hold: it was not there before
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
    std::lock_guard<std::mutex> lock(m_mutex);
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
    for (const size_t v : op_vertices) e.op_vertices.push_back(local.at(v));
    m_history.edits.push_back(std::move(e));
}

VolumeHistory VolumeRecorder::history() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    VolumeHistory h = m_history;
    std::stable_sort(h.edits.begin(), h.edits.end(), [](const auto& a, const auto& b) {
        return a.first_edit < b.first_edit;
    });
    return h;
}

size_t VolumeRecorder::size_estimate() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    size_t n = 0;
    for (const VolumeEdit& e : m_history.edits) {
        n += 8 + e.positions.size() * 28 + e.moved.size() * 26 + e.tets.size() * 6 +
             e.removed.size() * 2 + e.added.size() * 6 + e.op_vertices.size();
    }
    return n;
}

} // namespace wmtk::history
