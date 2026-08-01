#include <igl/is_edge_manifold.h>
#include <igl/writeDMAT.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/threading/parallel_for.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleUtils.hpp>

using namespace wmtk;

void TriMesh::Tuple::update_hash(const TriMesh& m)
{
    m_hash = m.m_tri_connectivity[m_fid].hash;
}

void TriMesh::Tuple::print_info()
{
    logger().trace("tuple: {} {} {}", m_vid, m_eid, m_fid);
}

size_t TriMesh::Tuple::eid(const TriMesh& m) const
{
    /*
     * The edge ID is constructed from the min face ID incident to the edge and in there the local
     * edge ID:
     * eid = 3 * min_fid + local_eid
     */

    // const std::optional<Tuple> t_opp = switch_face(m);
    // if (t_opp.has_value()) {
    //     size_t fid2 = t_opp.value().fid(m);
    //     size_t min_fid = std::min(m_fid, fid2);
    //     if (min_fid == fid2) {
    //         int i = m.m_tri_connectivity[fid2].find(m_vid);
    //         int j = m.m_tri_connectivity[fid2].find(switch_vertex(m).vid(m));
    //         return min_fid * 3 + 3 - i - j;
    //     }
    // }
    // return m_fid * 3 + m_eid;

    const size_t v_opp = switch_vertex(m).vid(m);

    const std::vector<size_t>& fids = m.m_vertex_connectivity[m_vid].m_conn_tris;

    // find face that contain m_vid and v_opp
    for (const size_t f : fids) {
        const auto& f_vids = m.m_tri_connectivity[f].m_indices;

        int local_v0 = -1;
        int local_v1 = -1;
        for (size_t i = 0; i < f_vids.size(); ++i) {
            if (f_vids[i] == m_vid) {
                local_v0 = (int)i;
            } else if (f_vids[i] == v_opp) {
                local_v1 = (int)i;
            }
        }
        if (local_v1 == -1) {
            continue;
        }
        assert(local_v0 != -1);
        size_t local_eid = 3 - (local_v0 + local_v1);
        // fids are sorted --> return smallest fid
        return 3 * f + local_eid;
    }

    log_and_throw_error(
        "Invalid Tuple: no face contains both vertices of the edge. Vertices are {}, {}, tuple "
        "face ID is {}",
        m_vid,
        v_opp,
        m_fid);
}


TriMesh::Tuple TriMesh::Tuple::switch_vertex(const TriMesh& m) const
{
    assert(is_valid(m));

    const size_t loc_v0 = m.m_tri_connectivity[m_fid][0];
    const size_t v1 = m.m_tri_connectivity[m_fid][1];
    const size_t v2 = m.m_tri_connectivity[m_fid][2];

    Tuple loc = *this;
    switch (m_eid) {
    case 0:
        assert(m_vid == v1 || m_vid == v2);
        loc.m_vid = m_vid == v1 ? v2 : v1;
        break;
    case 1:
        assert(m_vid == loc_v0 || m_vid == v2);
        loc.m_vid = m_vid == loc_v0 ? v2 : loc_v0;
        break;
    case 2:
        assert(m_vid == loc_v0 || m_vid == v1);
        loc.m_vid = m_vid == loc_v0 ? v1 : loc_v0;
        break;
    default:;
    }
    assert(loc.is_valid(m));

    return loc;
}

TriMesh::Tuple TriMesh::Tuple::switch_edge(const TriMesh& m) const
{
    assert(is_valid(m));

    const int lvid = m.m_tri_connectivity[m_fid].find(m_vid);
    assert(lvid == 0 || lvid == 1 || lvid == 2);

    Tuple loc = *this;
    switch (lvid) {
    case 0:
        assert(m_eid == 1 || m_eid == 2);
        loc.m_eid = m_eid == 1 ? 2 : 1;
        break;
    case 1:
        assert(m_eid == 0 || m_eid == 2);
        loc.m_eid = m_eid == 0 ? 2 : 0;
        break;
    case 2:
        assert(m_eid == 0 || m_eid == 1);
        loc.m_eid = m_eid == 0 ? 1 : 0;
        break;
    default:;
    }
    assert(loc.is_valid(m));
    return loc;
}

std::optional<TriMesh::Tuple> TriMesh::Tuple::switch_face(const TriMesh& m) const
{
    assert(is_valid(m));

    const size_t next_fid = m.m_tri_connectivity[m_fid].m_edge_next[m_eid];
    assert(next_fid != size_t(-1) && "radial cycle not built for this edge");

    // A boundary edge is a cycle of length one.
    if (next_fid == m_fid) return {};

    const size_t loc_v0 = m_vid;
    const size_t v1 = this->switch_vertex(m).m_vid;

    Tuple loc = *this;
    loc.m_fid = next_fid;

    const int lv0_2 = m.m_tri_connectivity[next_fid].find(loc_v0);
    const int lv1_2 = m.m_tri_connectivity[next_fid].find(v1);
    assert(lv0_2 != -1 && lv1_2 != -1);
    loc.m_eid = 3 - (lv0_2 + lv1_2);

    loc.update_hash(m);
    assert(loc.is_valid(m));
    return loc;
}

std::vector<TriMesh::Tuple> TriMesh::Tuple::switch_faces(const TriMesh& m) const
{
    assert(is_valid(m));

    const size_t loc_v0 = m_vid;
    const size_t v1 = this->switch_vertex(m).m_vid;

    // Walk the radial cycle, collecting every other face.
    std::vector<size_t> fids;
    for (size_t f = m.m_tri_connectivity[m_fid].m_edge_next[m_eid]; f != m_fid;) {
        assert(f != size_t(-1));
        fids.push_back(f);
        const int la = m.m_tri_connectivity[f].find(loc_v0);
        const int lb = m.m_tri_connectivity[f].find(v1);
        assert(la != -1 && lb != -1);
        f = m.m_tri_connectivity[f].m_edge_next[3 - la - lb];
    }

    // The cycle is in increasing fid order, so starting at the successor of m_fid yields
    // the faces above m_fid and then those below it. Rotate back to increasing order: that
    // is what this function has always returned, and callers such as collapse_edge_conn
    // take fids[0] and would otherwise pick a different face on a non-manifold edge.
    std::rotate(fids.begin(), std::min_element(fids.begin(), fids.end()), fids.end());

    std::vector<Tuple> faces;
    faces.reserve(fids.size());
    for (const size_t f : fids) {
        const int la = m.m_tri_connectivity[f].find(loc_v0);
        const int lb = m.m_tri_connectivity[f].find(v1);
        faces.emplace_back(loc_v0, 3 - la - lb, f, m);
    }
    return faces;
}

bool TriMesh::Tuple::is_valid(const TriMesh& m) const
{
    if (m_fid + 1 == 0) return false;
    if (m.m_vertex_connectivity[m_vid].m_is_removed || m.m_tri_connectivity[m_fid].m_is_removed) {
        // assert(false);
        return false;
    }
    // Condition 3: tuple m_hash check
    if (m_hash != m.m_tri_connectivity[m_fid].hash) {
        // assert(false);
        return false;
    }
#ifndef NDEBUG
    //  Condition 0: Elements exist
    assert(m_vid < m.vert_capacity());
    assert(m_eid <= 2);
    assert(m_fid <= m.tri_capacity());

    // Condition 1: tid and vid are consistent
    const int lvid = m.m_tri_connectivity[m_fid].find(m_vid);
    assert(lvid == 0 || lvid == 1 || lvid == 2);

    // Condition 2: eid is valid
    const size_t loc_v0 = m.m_tri_connectivity[m_fid][0];
    const size_t v1 = m.m_tri_connectivity[m_fid][1];
    const size_t v2 = m.m_tri_connectivity[m_fid][2];
    switch (m_eid) {
    case 0: assert(m_vid == v1 || m_vid == v2); break;
    case 1: assert(m_vid == loc_v0 || m_vid == v2); break;
    case 2: assert(m_vid == loc_v0 || m_vid == v1); break;
    default: assert(false);
    }
#endif

    return true;
}

namespace {

/// Local edge index of the edge whose endpoints sit at local vertex indices a and b.
/// The local edge is the one opposite the third local vertex, so the indices sum to 3.
inline size_t local_eid_from(int a, int b)
{
    assert(a >= 0 && a <= 2 && b >= 0 && b <= 2 && a != b);
    return static_cast<size_t>(3 - a - b);
}

} // namespace

void TriMesh::compute_edge_cycles(std::vector<std::array<size_t, 3>>& out) const
{
    out.assign(m_tri_connectivity.size(), {{size_t(-1), size_t(-1), size_t(-1)}});

    // (v_min, v_max, fid) for every corner of every live face, sorted. Grouping the sorted
    // run gives each edge's fan already in increasing fid order, which is the invariant the
    // cycle encodes. A sort beats a map here: one allocation, no node chasing, and the
    // result does not depend on any hash seed.
    std::vector<std::array<size_t, 3>> corners;
    corners.reserve(tri_capacity() * 3);
    for (size_t fid = 0; fid < tri_capacity(); ++fid) {
        if (m_tri_connectivity[fid].m_is_removed) continue;
        const auto& idx = m_tri_connectivity[fid].m_indices;
        for (int j = 0; j < 3; ++j) {
            const size_t a = idx[(j + 1) % 3];
            const size_t b = idx[(j + 2) % 3];
            corners.push_back({{std::min(a, b), std::max(a, b), fid}});
        }
    }
    std::sort(corners.begin(), corners.end());

    for (size_t i = 0; i < corners.size();) {
        size_t j = i;
        while (j < corners.size() && corners[j][0] == corners[i][0] &&
               corners[j][1] == corners[i][1]) {
            ++j;
        }
        // corners[i..j) is one edge's fan, in increasing fid order
        for (size_t k = i; k < j; ++k) {
            const size_t fid = corners[k][2];
            const size_t next = corners[(k + 1 == j) ? i : (k + 1)][2];
            const int la = m_tri_connectivity[fid].find(corners[k][0]);
            const int lb = m_tri_connectivity[fid].find(corners[k][1]);
            out[fid][local_eid_from(la, lb)] = next;
        }
        i = j;
    }
}

void TriMesh::compute_vertex_cycles(std::vector<std::array<size_t, 3>>& out) const
{
    out.assign(m_tri_connectivity.size(), {{size_t(-1), size_t(-1), size_t(-1)}});

    std::vector<size_t> successors;
    std::vector<int> local_vids;
    for (size_t vid = 0; vid < vert_capacity(); ++vid) {
        if (m_vertex_connectivity[vid].m_is_removed) continue;
        vertex_cycle_successors(vid, successors, local_vids);
        const std::vector<size_t>& fan = m_vertex_connectivity[vid].m_conn_tris;
        for (size_t i = 0; i < fan.size(); ++i) {
            out[fan[i]][local_vids[i]] = successors[i];
        }
    }
}

void TriMesh::vertex_cycle_successors(
    const size_t vid,
    std::vector<size_t>& successors,
    std::vector<int>& lv) const
{
    successors.clear();
    lv.clear();

    const std::vector<size_t>& fan = m_vertex_connectivity[vid].m_conn_tris;
    if (fan.empty()) return;

    // Union-find over positions in the fan. Two faces belong to the same component when
    // they share an edge containing vid, i.e. when vid is adjacent to the same other
    // vertex in both -- so grouping by that other vertex is enough, and it handles a
    // non-manifold edge correctly since all its faces land in the same group.
    //
    // A fan is a handful of faces, so every lookup below is a linear scan of a flat vector.
    // std::map here costs a node allocation per insert and this runs several times per
    // mesh operation, which measured as roughly a 2.5x slowdown of a whole simplification
    // pass -- far more than the scans it was saving.
    // thread_local, not members: this is a const method and the executors run operations on
    // several threads at once.
    thread_local std::vector<size_t> parent;
    thread_local std::vector<std::pair<size_t, size_t>> seen;
    thread_local std::vector<std::pair<size_t, size_t>> root_rep;
    thread_local std::vector<size_t> reps;

    parent.resize(fan.size());
    for (size_t i = 0; i < fan.size(); ++i) parent[i] = i;
    // `parent` has static storage duration, so it cannot be captured -- it is simply in
    // scope, which is all the lambda needs.
    const auto find_root = [](size_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };

    seen.clear(); // (other endpoint, fan position of the first face carrying it)
    lv.resize(fan.size());
    for (size_t i = 0; i < fan.size(); ++i) {
        lv[i] = m_tri_connectivity[fan[i]].find(vid);
        assert(lv[i] != -1);
        const auto& idx = m_tri_connectivity[fan[i]].m_indices;
        for (int k = 1; k <= 2; ++k) {
            const size_t other = idx[(lv[i] + k) % 3];
            bool found = false;
            for (const auto& [o, pos] : seen) {
                if (o != other) continue;
                found = true;
                const size_t ra = find_root(i);
                const size_t rb = find_root(pos);
                if (ra != rb) parent[ra] = rb;
                break;
            }
            if (!found) seen.emplace_back(other, i);
        }
    }

    // Each component is named by its smallest fid. fan is sorted, so the first face seen
    // for a root is already the smallest.
    root_rep.clear(); // (root, representative fid)
    for (size_t i = 0; i < fan.size(); ++i) {
        const size_t r = find_root(i);
        bool found = false;
        for (const auto& [root, rep] : root_rep) {
            if (root == r) {
                found = true;
                break;
            }
        }
        if (!found) root_rep.emplace_back(r, fan[i]);
    }

    reps.clear();
    for (const auto& [root, rep] : root_rep) reps.push_back(rep);
    std::sort(reps.begin(), reps.end());

    successors.resize(fan.size());
    for (size_t i = 0; i < fan.size(); ++i) {
        const size_t r = find_root(i);
        size_t rep = size_t(-1);
        for (const auto& [root, rp] : root_rep) {
            if (root == r) {
                rep = rp;
                break;
            }
        }
        assert(rep != size_t(-1));
        const auto it = std::lower_bound(reps.begin(), reps.end(), rep);
        assert(it != reps.end() && *it == rep);
        const size_t pos = size_t(it - reps.begin());
        successors[i] = reps[(pos + 1) % reps.size()];
    }
}

void TriMesh::rebuild_edge_cycle(const size_t v0, const size_t v1)
{
    // Intersect the two endpoint fans, which are sorted, so the result comes out in
    // increasing fid order already. Done by hand into a reused buffer rather than through
    // get_incident_fids_for_edge, which returns a fresh vector -- this runs a few dozen
    // times per mesh operation.
    thread_local std::vector<size_t> fids;
    fids.clear();
    {
        const std::vector<size_t>& a = m_vertex_connectivity[v0].m_conn_tris;
        const std::vector<size_t>& b = m_vertex_connectivity[v1].m_conn_tris;
        std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(fids));
    }
    for (size_t i = 0; i < fids.size(); ++i) {
        const size_t fid = fids[i];
        const int la = m_tri_connectivity[fid].find(v0);
        const int lb = m_tri_connectivity[fid].find(v1);
        assert(la != -1 && lb != -1);
        m_tri_connectivity[fid].m_edge_next[local_eid_from(la, lb)] = fids[(i + 1) % fids.size()];
    }
}

void TriMesh::rebuild_vertex_cycle(const size_t vid)
{
    const std::vector<size_t>& fan = m_vertex_connectivity[vid].m_conn_tris;
    if (fan.empty()) return;

    thread_local std::vector<size_t> successors;
    thread_local std::vector<int> lv;
    vertex_cycle_successors(vid, successors, lv);

    for (size_t i = 0; i < fan.size(); ++i) {
        m_tri_connectivity[fan[i]].m_vert_next_component[lv[i]] = successors[i];
    }
}

void TriMesh::rebuild_cycles_around(const std::vector<size_t>& fids)
{
    // Flat vectors deduplicated by sort+unique rather than std::set: a star is a handful of
    // faces, and a node allocation per insert dominated everything else here.
    thread_local std::vector<std::pair<size_t, size_t>> edges;
    thread_local std::vector<size_t> verts;
    edges.clear();
    verts.clear();

    for (const size_t fid : fids) {
        if (fid >= m_tri_connectivity.size() || m_tri_connectivity[fid].m_is_removed) continue;
        const auto& idx = m_tri_connectivity[fid].m_indices;
        for (int j = 0; j < 3; ++j) {
            verts.push_back(idx[j]);
            const size_t a = idx[(j + 1) % 3];
            const size_t b = idx[(j + 2) % 3];
            edges.emplace_back(std::min(a, b), std::max(a, b));
        }
    }
    std::sort(edges.begin(), edges.end());
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
    std::sort(verts.begin(), verts.end());
    verts.erase(std::unique(verts.begin(), verts.end()), verts.end());

    // The callees reuse buffers of their own, distinct from these two, so iterating in
    // place is safe.
    for (const auto& [a, b] : edges) rebuild_edge_cycle(a, b);
    for (const size_t vid : verts) rebuild_vertex_cycle(vid);
}

void TriMesh::rebuild_cycles_for(
    std::vector<std::pair<size_t, size_t>>& edges,
    std::vector<size_t>& vids)
{
    std::sort(edges.begin(), edges.end());
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
    std::sort(vids.begin(), vids.end());
    vids.erase(std::unique(vids.begin(), vids.end()), vids.end());

    // An edge with no incident faces left simply writes nothing, and a removed or faceless
    // vertex returns immediately, so stale entries need no filtering here.
    for (const auto& [a, b] : edges) rebuild_edge_cycle(a, b);
    for (const size_t vid : vids) {
        if (vid >= m_vertex_connectivity.size() || m_vertex_connectivity[vid].m_is_removed) {
            continue;
        }
        rebuild_vertex_cycle(vid);
    }
}

void TriMesh::rebuild_all_cycles()
{
    std::vector<std::array<size_t, 3>> edge_next;
    std::vector<std::array<size_t, 3>> vert_next;
    compute_edge_cycles(edge_next);
    compute_vertex_cycles(vert_next);

    for (size_t fid = 0; fid < m_tri_connectivity.size(); ++fid) {
        m_tri_connectivity[fid].m_edge_next = edge_next[fid];
        m_tri_connectivity[fid].m_vert_next_component = vert_next[fid];
    }
}

size_t TriMesh::edge_valence(const Tuple& t) const
{
    const size_t fid = t.fid(*this);
    const size_t eid = t.local_eid(*this);
    assert(m_tri_connectivity[fid].m_edge_next[eid] != size_t(-1));

    const size_t v0 = t.vid(*this);
    const size_t v1 = t.switch_vertex(*this).vid(*this);

    size_t count = 1;
    for (size_t f = m_tri_connectivity[fid].m_edge_next[eid]; f != fid; ++count) {
        const int la = m_tri_connectivity[f].find(v0);
        const int lb = m_tri_connectivity[f].find(v1);
        assert(la != -1 && lb != -1);
        f = m_tri_connectivity[f].m_edge_next[local_eid_from(la, lb)];
    }
    return count;
}

bool TriMesh::is_boundary_edge(const Tuple& t) const
{
    // A boundary edge is a cycle of length one, so its successor is the face itself.
    return m_tri_connectivity[t.fid(*this)].m_edge_next[t.local_eid(*this)] == t.fid(*this);
}

#ifdef WMTK_DEBUG_BRUTE_FORCE_OPS

namespace {

/// One face per line, rotated to start at its smallest vid. Rotation keeps the cyclic
/// order -- and so the orientation -- while removing the choice of starting corner, which
/// the operations are free to change and which nothing downstream depends on.
void append_face(std::string& s, std::array<size_t, 3> vids)
{
    const size_t start = std::min_element(vids.begin(), vids.end()) - vids.begin();
    s += "f";
    for (int k = 0; k < 3; ++k) {
        s += " " + std::to_string(vids[(start + k) % 3]);
    }
    s += "\n";
}

std::string canonical_form(
    const std::vector<char>& vert_alive,
    const std::vector<std::array<size_t, 3>>& faces,
    const std::vector<char>& face_alive)
{
    std::string s;
    for (size_t v = 0; v < vert_alive.size(); ++v) {
        if (vert_alive[v]) s += "v " + std::to_string(v) + "\n";
    }
    for (size_t f = 0; f < faces.size(); ++f) {
        if (face_alive[f]) append_face(s, faces[f]);
    }
    return s;
}

} // namespace

std::string TriMesh::debug_canonical_form() const
{
    std::vector<char> vert_alive(vert_capacity(), 0);
    for (size_t v = 0; v < vert_capacity(); ++v) {
        vert_alive[v] = !m_vertex_connectivity[v].m_is_removed;
    }
    std::vector<std::array<size_t, 3>> faces(tri_capacity());
    std::vector<char> face_alive(tri_capacity(), 0);
    for (size_t f = 0; f < tri_capacity(); ++f) {
        faces[f] = m_tri_connectivity[f].m_indices;
        face_alive[f] = !m_tri_connectivity[f].m_is_removed;
    }
    return canonical_form(vert_alive, faces, face_alive);
}

std::string TriMesh::debug_reference_collapse(const size_t v_removed, const size_t v_kept) const
{
    std::vector<char> vert_alive(vert_capacity(), 0);
    for (size_t v = 0; v < vert_capacity(); ++v) {
        vert_alive[v] = !m_vertex_connectivity[v].m_is_removed;
    }
    std::vector<std::array<size_t, 3>> faces(tri_capacity());
    std::vector<char> face_alive(tri_capacity(), 0);
    for (size_t f = 0; f < tri_capacity(); ++f) {
        faces[f] = m_tri_connectivity[f].m_indices;
        face_alive[f] = !m_tri_connectivity[f].m_is_removed;
    }

    // Drop every face carrying both endpoints -- those are the ones the collapse degenerates
    // -- then rename the removed endpoint everywhere else.
    for (size_t f = 0; f < faces.size(); ++f) {
        if (!face_alive[f]) continue;
        const bool has_removed =
            std::find(faces[f].begin(), faces[f].end(), v_removed) != faces[f].end();
        const bool has_kept = std::find(faces[f].begin(), faces[f].end(), v_kept) != faces[f].end();
        if (has_removed && has_kept) {
            face_alive[f] = 0;
        } else if (has_removed) {
            for (size_t& v : faces[f]) {
                if (v == v_removed) v = v_kept;
            }
        }
    }
    vert_alive[v_removed] = 0;

    // Two faces can now carry the same triple. Keep the smallest fid of each group, which
    // is what collapse_edge_conn does.
    //
    // Restricted to faces at the surviving vertex, and not the whole mesh, because that is
    // the contract: a collapse merges the duplicates it creates, it does not garbage-collect
    // ones that were already there. A pre-existing duplicate pair elsewhere must survive
    // untouched -- Thingi10K 314748 has exactly one, which is how this came up.
    {
        std::map<std::array<size_t, 3>, size_t> first_with_key;
        for (size_t f = 0; f < faces.size(); ++f) {
            if (!face_alive[f]) continue;
            if (std::find(faces[f].begin(), faces[f].end(), v_kept) == faces[f].end()) continue;
            std::array<size_t, 3> key = faces[f];
            std::sort(key.begin(), key.end());
            if (!first_with_key.try_emplace(key, f).second) face_alive[f] = 0;
        }
    }

    // Anything left with no face at all is retired.
    {
        std::vector<char> referenced(vert_alive.size(), 0);
        for (size_t f = 0; f < faces.size(); ++f) {
            if (!face_alive[f]) continue;
            for (const size_t v : faces[f]) referenced[v] = 1;
        }
        for (size_t v = 0; v < vert_alive.size(); ++v) {
            if (!referenced[v]) vert_alive[v] = 0;
        }
    }

    return canonical_form(vert_alive, faces, face_alive);
}

std::string TriMesh::debug_reference_swap(
    const size_t v0,
    const size_t v1,
    const size_t fa,
    const size_t fb) const
{
    std::vector<char> vert_alive(vert_capacity(), 0);
    for (size_t v = 0; v < vert_capacity(); ++v) {
        vert_alive[v] = !m_vertex_connectivity[v].m_is_removed;
    }
    std::vector<std::array<size_t, 3>> faces(tri_capacity());
    std::vector<char> face_alive(tri_capacity(), 0);
    for (size_t f = 0; f < tri_capacity(); ++f) {
        faces[f] = m_tri_connectivity[f].m_indices;
        face_alive[f] = !m_tri_connectivity[f].m_is_removed;
    }

    const auto apex = [&](size_t f) {
        for (const size_t v : faces[f]) {
            if (v != v0 && v != v1) return v;
        }
        assert(false);
        return size_t(-1);
    };
    const size_t a = apex(fa);
    const size_t b = apex(fb);

    // Substituting in place keeps each vertex in the slot it occupied, which is how
    // swap_edge does it and is what preserves the orientation of both triangles.
    for (size_t& v : faces[fa]) {
        if (v == v1) v = b;
    }
    for (size_t& v : faces[fb]) {
        if (v == v0) v = a;
    }

    return canonical_form(vert_alive, faces, face_alive);
}

std::string TriMesh::debug_reference_split_face(
    const size_t fid,
    const size_t v0,
    const size_t v1,
    const size_t v2,
    const size_t new_v,
    const size_t tri_cap) const
{
    std::vector<char> vert_alive(std::max(vert_capacity(), new_v + 1), 0);
    for (size_t v = 0; v < vert_capacity(); ++v) {
        vert_alive[v] = !m_vertex_connectivity[v].m_is_removed;
    }
    vert_alive[new_v] = 1;

    std::vector<std::array<size_t, 3>> faces(tri_cap);
    std::vector<char> face_alive(tri_cap, 0);
    for (size_t f = 0; f < tri_cap; ++f) {
        faces[f] = m_tri_connectivity[f].m_indices;
        face_alive[f] = !m_tri_connectivity[f].m_is_removed;
    }

    // The face becomes three, each the original with one vertex replaced by the new one.
    // Replacing in place preserves orientation. The slot order matters: split_face keeps
    // the v0 child in the original fid and appends the v1 and v2 children in that order,
    // which is the order get_next_empty_slot_t hands out.
    const std::array<size_t, 3> original = faces[fid];
    const auto child = [&original](size_t replaced, size_t with) {
        std::array<size_t, 3> c = original;
        for (size_t& v : c) {
            if (v == replaced) v = with;
        }
        return c;
    };

    faces[fid] = child(v0, new_v);
    faces.push_back(child(v1, new_v));
    face_alive.push_back(1);
    faces.push_back(child(v2, new_v));
    face_alive.push_back(1);

    return canonical_form(vert_alive, faces, face_alive);
}

std::string TriMesh::debug_reference_split(
    const size_t v0,
    const size_t v1,
    const size_t new_v,
    const size_t tri_cap) const
{
    std::vector<char> vert_alive(std::max(vert_capacity(), new_v + 1), 0);
    for (size_t v = 0; v < vert_capacity(); ++v) {
        vert_alive[v] = !m_vertex_connectivity[v].m_is_removed;
    }
    vert_alive[new_v] = 1;

    std::vector<std::array<size_t, 3>> faces(tri_cap);
    std::vector<char> face_alive(tri_cap, 0);
    for (size_t f = 0; f < tri_cap; ++f) {
        faces[f] = m_tri_connectivity[f].m_indices;
        face_alive[f] = !m_tri_connectivity[f].m_is_removed;
    }

    // Each face on the edge becomes two: the original with v1 renamed to new_v, and a new
    // one with v0 renamed to new_v. Same rule split_edge applies, and the new faces are
    // appended in increasing order of the face they came from, which is the order
    // get_next_empty_slot_t hands out.
    std::vector<std::array<size_t, 3>> appended;
    for (size_t f = 0; f < tri_cap; ++f) {
        if (!face_alive[f]) continue;
        const bool has_v0 = std::find(faces[f].begin(), faces[f].end(), v0) != faces[f].end();
        const bool has_v1 = std::find(faces[f].begin(), faces[f].end(), v1) != faces[f].end();
        if (!has_v0 || !has_v1) continue;

        std::array<size_t, 3> child = faces[f];
        for (size_t& v : child) {
            if (v == v0) v = new_v;
        }
        appended.push_back(child);

        for (size_t& v : faces[f]) {
            if (v == v1) v = new_v;
        }
    }
    for (const auto& child : appended) {
        faces.push_back(child);
        face_alive.push_back(1);
    }

    return canonical_form(vert_alive, faces, face_alive);
}

#endif // WMTK_DEBUG_BRUTE_FORCE_OPS

size_t TriMesh::vertex_component_count(const size_t vid) const
{
    const std::vector<size_t>& fan = m_vertex_connectivity[vid].m_conn_tris;
    if (fan.empty()) return 0;

    // Any face's entry names the representative of the *next* component, so walking from
    // there visits every representative exactly once.
    const int lv0 = m_tri_connectivity[fan[0]].find(vid);
    assert(lv0 != -1);
    const size_t first = m_tri_connectivity[fan[0]].m_vert_next_component[lv0];
    assert(first != size_t(-1));

    size_t count = 0;
    size_t f = first;
    do {
        ++count;
        const int lv = m_tri_connectivity[f].find(vid);
        assert(lv != -1);
        f = m_tri_connectivity[f].m_vert_next_component[lv];
    } while (f != first);
    return count;
}

std::optional<TriMesh::Tuple> TriMesh::switch_component(const Tuple& t) const
{
    assert(t.is_valid(*this));

    const size_t vid = t.vid(*this);
    const int lv = m_tri_connectivity[t.fid(*this)].find(vid);
    assert(lv != -1);
    const size_t next_rep = m_tri_connectivity[t.fid(*this)].m_vert_next_component[lv];
    assert(next_rep != size_t(-1));

    // With a single component the cycle is a self-loop on its representative, and there is
    // nowhere else to go.
    const int lv_next = m_tri_connectivity[next_rep].find(vid);
    assert(lv_next != -1);
    if (m_tri_connectivity[next_rep].m_vert_next_component[lv_next] == next_rep) {
        return {};
    }

    // Same convention as get_one_ring_tris_for_vertex: the tuple owns `vid`.
    return Tuple(vid, (lv_next + 2) % 3, next_rep, *this);
}

// a valid mesh can have triangles that are is_removed == true
bool wmtk::TriMesh::check_mesh_connectivity_validity() const
{
    std::vector<std::vector<size_t>> conn_tris(vert_capacity());
    for (size_t i = 0; i < tri_capacity(); i++) {
        if (m_tri_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 3; j++) conn_tris[m_tri_connectivity[i][j]].push_back(i);
    }

    for (unsigned i = 0; i < vert_capacity(); ++i)
        std::sort(conn_tris[i].begin(), conn_tris[i].end());

    // check conn_tets duplication, order, amount ...
    for (size_t i = 0; i < vert_capacity(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;

        assert(
            m_vertex_connectivity[i].m_conn_tris == conn_tris[i] &&
            "m_vertex_connectivity[i].m_conn_tris!=conn_tris[i]");
        if (m_vertex_connectivity[i].m_conn_tris != conn_tris[i]) return false;
    }

    // The radial and component cycles are a canonical function of the connectivity above,
    // so recomputing them from scratch and comparing catches any operation that failed to
    // maintain them. This is the drift detector for the whole non-manifold machinery.
    {
        std::vector<std::array<size_t, 3>> edge_next;
        std::vector<std::array<size_t, 3>> vert_next;
        compute_edge_cycles(edge_next);
        compute_vertex_cycles(vert_next);

        for (size_t fid = 0; fid < tri_capacity(); ++fid) {
            if (m_tri_connectivity[fid].m_is_removed) continue;
            assert(
                m_tri_connectivity[fid].m_edge_next == edge_next[fid] &&
                "stale radial cycle: an operation did not rebuild it");
            assert(
                m_tri_connectivity[fid].m_vert_next_component == vert_next[fid] &&
                "stale component cycle: an operation did not rebuild it");
            if (m_tri_connectivity[fid].m_edge_next != edge_next[fid]) return false;
            if (m_tri_connectivity[fid].m_vert_next_component != vert_next[fid]) return false;
        }
    }
    return true;
}

bool wmtk::TriMesh::check_edge_manifold() const
{
    for (const Tuple& e : get_edges()) {
        if (edge_valence(e) > 2) return false;
    }
    return true;
}

bool TriMesh::split_edge(const Tuple& t, std::vector<Tuple>& new_tris)
{
    if (!split_edge_before(t)) {
        return false;
    }
    if (!t.is_valid(*this)) {
        return false;
    }
    // get local eid for return tuple construction
    const size_t eid = t.local_eid(*this);
    const size_t t_fid = t.fid(*this);
    // get the vids
    const size_t vid1 = t.vid(*this);
    const size_t vid2 = t.switch_vertex(*this).vid(*this);
    std::vector<size_t> fids;
    fids.reserve(2);
    std::vector<size_t> vid3s;
    vid3s.reserve(2);

    // find neighboring faces sharing vid1 and vid2
    for (const size_t f : m_vertex_connectivity[vid1].m_conn_tris) {
        size_t v3;
        bool v2_found = false;
        for (const size_t v : m_tri_connectivity[f].m_indices) {
            if (v == vid2) {
                v2_found = true;
                continue;
            }
            if (v != vid1) {
                v3 = v;
            }
        }

        if (!v2_found) {
            continue;
        }
        fids.push_back(f);
        vid3s.push_back(v3);
    }
    assert(!(fids.empty() || vid3s.empty()));

    // record the vids that will be modified for roll backs on failure
    std::vector<std::pair<size_t, VertexConnectivity>> old_vertices(2);
    std::vector<std::pair<size_t, TriangleConnectivity>> old_tris;
    old_vertices[0] = std::make_pair(vid1, m_vertex_connectivity[vid1]);
    old_vertices[1] = std::make_pair(vid2, m_vertex_connectivity[vid2]);
    for (size_t i = 0; i < vid3s.size(); ++i) {
        old_vertices.emplace_back(vid3s[i], m_vertex_connectivity[vid3s[i]]);
        old_tris.emplace_back(fids[i], m_tri_connectivity[fids[i]]);
    }

#ifdef WMTK_DEBUG_BRUTE_FORCE_OPS
    if (!check_mesh_connectivity_validity()) {
        log_and_throw_error("split_edge: connectivity was already invalid on entry");
    }
    // Before the slots are reserved: past this point tri_capacity() counts them, and they
    // are live but unfilled.
    const size_t bf_tri_cap = tri_capacity();
#endif

    const size_t new_vid = get_next_empty_slot_v();
    std::vector<size_t> new_fids(fids.size());
    for (size_t i = 0; i < fids.size(); ++i) {
        new_fids[i] = get_next_empty_slot_t();
    }

    // abort before mutating if we ran out of preallocated slots; mark any reserved
    // slots removed so they don't become live-but-empty phantoms.
    {
        constexpr size_t INVALID_SLOT = static_cast<size_t>(-1);
        bool slots_ok = (new_vid != INVALID_SLOT);
        for (size_t f : new_fids)
            if (f == INVALID_SLOT) slots_ok = false;
        if (!slots_ok) {
            if (new_vid != INVALID_SLOT) m_vertex_connectivity[new_vid].m_is_removed = true;
            for (size_t f : new_fids)
                if (f != INVALID_SLOT) m_tri_connectivity[f].m_is_removed = true;
            return false;
        }
    }

#ifdef WMTK_DEBUG_BRUTE_FORCE_OPS
    const std::string brute_force_expected = debug_reference_split(vid1, vid2, new_vid, bf_tri_cap);
#endif

    // first work on the vids
    // the old triangles are connected to the vertex of t
    // fid1_vid3
    for (size_t i = 0; i < fids.size(); ++i) {
        m_vertex_connectivity[vid3s[i]].m_conn_tris.push_back(new_fids[i]);
        std::sort(
            m_vertex_connectivity[vid3s[i]].m_conn_tris.begin(),
            m_vertex_connectivity[vid3s[i]].m_conn_tris.end());
    }

    // vid2
    for (size_t i = 0; i < fids.size(); ++i) {
        vector_erase(m_vertex_connectivity[vid2].m_conn_tris, fids[i]);
        m_vertex_connectivity[vid2].m_conn_tris.push_back(new_fids[i]);
    }

    std::sort(
        m_vertex_connectivity[vid2].m_conn_tris.begin(),
        m_vertex_connectivity[vid2].m_conn_tris.end());

    // new_vid
    for (size_t i = 0; i < fids.size(); ++i) {
        m_vertex_connectivity[new_vid].m_conn_tris.push_back(fids[i]);
    }
    for (size_t i = 0; i < fids.size(); ++i) {
        m_vertex_connectivity[new_vid].m_conn_tris.push_back(new_fids[i]);
    }
    std::sort(
        m_vertex_connectivity[new_vid].m_conn_tris.begin(),
        m_vertex_connectivity[new_vid].m_conn_tris.end());


    // now the triangles
    // need to update the hash
    // fid1 fid2 update m_indices and hash
    for (size_t i = 0; i < fids.size(); ++i) {
        const size_t fid = fids[i];
        const size_t vid3 = vid3s[i];
        const size_t new_fid = new_fids[i];

        const size_t tj = m_tri_connectivity[fid].find(vid2);
        m_tri_connectivity[fid].m_indices[tj] = new_vid;
        m_tri_connectivity[fid].hash++;
        const size_t ti = m_tri_connectivity[fid].find(vid1);
        const size_t tk = m_tri_connectivity[fid].find(vid3);
        // new_fid1 m_indices in same order
        m_tri_connectivity[new_fid].m_indices[ti] = new_vid;
        m_tri_connectivity[new_fid].m_indices[tj] = vid2;
        m_tri_connectivity[new_fid].m_indices[tk] = vid3;
        m_tri_connectivity[new_fid].hash++;
    }

    // The two halves of the split edge inherit the parent's fan, and each spoke to a vid3
    // is a fresh manifold edge -- but the pre-existing edge (vid2,vid3) also changed
    // membership, since the face carrying it was renamed from fid to new_fid. Rebuilding
    // over the whole affected star covers all of that without case analysis.
    std::vector<size_t> touched = fids;
    touched.insert(touched.end(), new_fids.begin(), new_fids.end());
    rebuild_cycles_around(touched);

#ifdef WMTK_DEBUG_BRUTE_FORCE_OPS
    if (debug_canonical_form() != brute_force_expected) {
        log_and_throw_error(
            "split_edge({},{}) disagrees with the brute-force reference.\nexpected:\n{}\ngot:\n{}",
            vid1,
            vid2,
            brute_force_expected,
            debug_canonical_form());
    }
#endif

    // make the new tuple
    size_t min_fid = m_vertex_connectivity[new_vid].m_conn_tris[0]; // making use of sorted vector


    int l = m_tri_connectivity[min_fid].find(new_vid);
    auto new_vertex = Tuple(new_vid, (l + 2) % 3, min_fid, *this);
    auto return_tuple = Tuple(vid1, eid, t_fid, *this);
    assert(new_vertex.is_valid(*this));
    assert(return_tuple.is_valid(*this));

    new_tris = get_one_ring_tris_for_vertex(new_vertex);
    start_protect_attributes();

    // roll back if not successful
    if (!split_edge_after(return_tuple) || !invariants(new_tris)) {
        // rollback topo
        // restore old v, t
        for (const auto& old_v : old_vertices) {
            m_vertex_connectivity[old_v.first] = old_v.second;
        }
        for (const auto& old_t : old_tris) {
            m_tri_connectivity[old_t.first] = old_t.second;
        }
        // erase new_vid new_fids
        m_vertex_connectivity[new_vid].m_conn_tris.clear();
        m_vertex_connectivity[new_vid].m_is_removed = true;
        for (size_t i = 0; i < fids.size(); ++i) {
            m_tri_connectivity[new_fids[i]].m_is_removed = true;
        }
        // Restoring old_tris brings back their own cycles, but the forward pass also
        // rewrote the cycles of faces outside that set -- the far side of every (vid2,vid3)
        // edge, for one. Recomputing over the same region against the restored connectivity
        // puts all of them back.
        rebuild_cycles_around(touched);
        rollback_protected_attributes();
        return false;
    }
    release_protect_attributes();
    return true;
}

// bool TriMesh::collapse_edge(const Tuple& loc0, std::vector<Tuple>& new_tris)
// {
//     if (!collapse_edge_before(loc0)) {
//         return false;
//     }
//     // get fid for the return tuple
//     // take the face that shares the same vertex the loc0 tuple is pointing to
//     // or if that face doesn't exit
//     // take the face that shares the same vertex of loc0
//     auto new_fid =
//         loc0.switch_vertex(*this).switch_edge(*this).switch_face(*this).has_value()
//             ?
//             (loc0.switch_vertex(*this).switch_edge(*this).switch_face(*this).value()).fid(*this)
//             : (loc0.switch_edge(*this).switch_face(*this).value()).fid(*this);
//     // get the vids
//     size_t vid1 = loc0.vid(*this);
//     size_t vid2 = switch_vertex(loc0).vid(*this);

//     // record the vids that will be erased for roll backs on failure
//     std::vector<std::pair<size_t, VertexConnectivity>> old_vertices(2);
//     old_vertices[0] = std::make_pair(vid1, m_vertex_connectivity[vid1]);
//     old_vertices[1] = std::make_pair(vid2, m_vertex_connectivity[vid2]);

//     // get the fids
//     auto n1_fids = m_vertex_connectivity[vid1].m_conn_tris;

//     auto n2_fids = m_vertex_connectivity[vid2].m_conn_tris;

//     // get the fids that will be modified
//     auto n12_intersect_fids = set_intersection(n1_fids, n2_fids);
//     // check if the triangles intersection is the one adjcent to the edge
//     size_t test_fid1 = loc0.fid(*this);
//     TriMesh::Tuple loc1 = switch_face(loc0).value_or(loc0);
//     size_t test_fid2 = loc1.fid(*this);
//     //"faces at the edge is not correct"
//     assert(
//         vector_contains(n12_intersect_fids, test_fid1) &&
//         vector_contains(n12_intersect_fids, test_fid2));
//     // now mark the vertices as removed so the assertion for tuple validity in switch operations
//     // won't fail
//     m_vertex_connectivity[vid1].m_is_removed = true;
//     m_vertex_connectivity[vid2].m_is_removed = true;
//     for (size_t fid : n12_intersect_fids) {
//         m_tri_connectivity[fid].m_is_removed = true;
//     }

//     std::vector<size_t> n12_union_fids;
//     std::set_union(
//         n1_fids.begin(),
//         n1_fids.end(),
//         n2_fids.begin(),
//         n2_fids.end(),
//         std::back_inserter(n12_union_fids));

//     // record the fids that will be modified/erased for roll back on failure
//     vector_unique(n12_union_fids);
//     std::vector<std::pair<size_t, TriangleConnectivity>> old_tris(n12_union_fids.size());

//     for (int i = 0; i < old_tris.size(); i++) {
//         size_t fid = n12_union_fids[i];
//         old_tris[i] = std::make_pair(fid, m_tri_connectivity[fid]);
//         m_tri_connectivity[fid].hash++;
//     }
//     // modify the triangles
//     // the m_conn_tris needs to be sorted
//     size_t new_vid = get_next_empty_slot_v();
//     for (size_t fid : n1_fids) {
//         if (m_tri_connectivity[fid].m_is_removed)
//             continue;
//         else {
//             int j = m_tri_connectivity[fid].find(vid1);
//             m_tri_connectivity[fid].m_indices[j] = new_vid;
//         }
//     }
//     for (size_t fid : n2_fids) {
//         if (m_tri_connectivity[fid].m_is_removed)
//             continue;
//         else {
//             int j = m_tri_connectivity[fid].find(vid2);
//             m_tri_connectivity[fid].m_indices[j] = new_vid;
//         }
//     }

//     // now work on vids
//     // add in the new vertex

//     for (size_t fid : n12_union_fids) {
//         if (m_tri_connectivity[fid].m_is_removed)
//             continue;
//         else
//             m_vertex_connectivity[new_vid].m_conn_tris.push_back(fid);
//     }
//     // This is sorting too, and it is important to sort
//     vector_unique(m_vertex_connectivity[new_vid].m_conn_tris);

//     // remove the erased fids from the vertices' (the one of the triangles that is not the end
//     // points of the edge) connectivity list
//     std::vector<std::pair<size_t, size_t>> same_edge_vid_fid;
//     for (size_t fid : n12_intersect_fids) {
//         auto f_vids = m_tri_connectivity[fid].m_indices;
//         for (size_t f_vid : f_vids) {
//             if (f_vid != vid1 && f_vid != vid2) {
//                 same_edge_vid_fid.emplace_back(f_vid, fid);
//                 assert(vector_contains(m_vertex_connectivity[f_vid].m_conn_tris, fid));
//                 vector_erase(m_vertex_connectivity[f_vid].m_conn_tris, fid);
//             }
//         }
//     }

//     // ? ? tuples changes. this needs to be done before post check since checked are done on tuples
//     // update the old tuple version number
//     // create an edge tuple for each changed edge
//     // call back check will be done on this vector of tuples

//     assert(m_vertex_connectivity[new_vid].m_conn_tris.size() != 0);

//     const size_t gfid = m_vertex_connectivity[new_vid].m_conn_tris[0];
//     int j = m_tri_connectivity[gfid].find(new_vid);
//     auto new_t = Tuple(new_vid, (j + 2) % 3, gfid, *this);
//     int j_ret = m_tri_connectivity[new_fid].find(new_vid);
//     auto return_t = Tuple(new_vid, (j_ret + 2) % 3, new_fid, *this);
//     assert(new_t.is_valid(*this));
//     new_tris = get_one_ring_tris_for_vertex(new_t);

//     start_protect_attributes();

//     if (!collapse_edge_after(return_t) || !invariants(new_tris)) {
//         // if call back check failed roll back
//         // restore the changes for connected triangles and vertices
//         // resotre the version-number
//         // removed restore to false

//         for (auto rollback : old_tris) {
//             size_t fid = rollback.first;
//             m_tri_connectivity[fid] = rollback.second;
//         }
//         for (auto rollback : old_vertices) {
//             size_t vid = rollback.first;
//             m_vertex_connectivity[vid] = rollback.second;
//         }
//         m_vertex_connectivity[new_vid].m_conn_tris.clear();
//         m_vertex_connectivity[new_vid].m_is_removed = true;
//         for (auto vid_fid : same_edge_vid_fid) {
//             size_t vid = vid_fid.first;
//             size_t fid = vid_fid.second;
//             m_vertex_connectivity[vid].m_conn_tris.push_back(fid);
//             std::sort(
//                 m_vertex_connectivity[vid].m_conn_tris.begin(),
//                 m_vertex_connectivity[vid].m_conn_tris.end());
//         }
//         for (size_t fid : n12_intersect_fids) {
//             m_tri_connectivity[fid].m_is_removed = false;
//         }

//         rollback_protected_attributes();
//         return false;
//     }
//     release_protect_attributes();
//     return true;
// }

bool TriMesh::collapse_edge(const Tuple& loc0, std::vector<Tuple>& new_tris)
{
    if (!collapse_edge_before(loc0)) {
        return false;
    }

    // A collapse consuming every face at both endpoints -- the two edges of a lone triangle,
    // say -- would leave the surviving vertex live but faceless, which no tuple can point
    // at. The link condition used to rule this out on the way in; refuse it explicitly so
    // it stays ruled out when the link condition is off.
    {
        const size_t vid1 = loc0.vid(*this);
        const size_t vid2 = loc0.switch_vertex(*this).vid(*this);
        const auto& n1 = m_vertex_connectivity[vid1].m_conn_tris;
        const auto& n2 = m_vertex_connectivity[vid2].m_conn_tris;
        const size_t shared = set_intersection(n1, n2).size();
        if (shared == n1.size() && shared == n2.size()) {
            return false;
        }
    }

    Tuple return_t;
    size_t new_vid;
    std::vector<std::pair<size_t, TriangleConnectivity>> old_tris;
    std::vector<std::pair<size_t, VertexConnectivity>> old_vertices;
    std::vector<std::pair<size_t, size_t>> same_edge_vid_fid;
    std::vector<size_t> n12_intersect_fids;

#ifdef WMTK_DEBUG_BRUTE_FORCE_OPS
    if (!check_mesh_connectivity_validity()) {
        log_and_throw_error("collapse_edge: connectivity was already invalid on entry");
    }
    // collapse_edge_conn retires the tuple's own vertex and keeps the other.
    const size_t bf_removed = loc0.vid(*this);
    const size_t bf_kept = loc0.switch_vertex(*this).vid(*this);
    const std::string brute_force_expected = debug_reference_collapse(bf_removed, bf_kept);
#endif

    collapse_edge_conn(
        loc0,
        new_tris,
        return_t,
        new_vid,
        old_tris,
        old_vertices,
        same_edge_vid_fid,
        n12_intersect_fids);

#ifdef WMTK_DEBUG_BRUTE_FORCE_OPS
    if (debug_canonical_form() != brute_force_expected) {
        log_and_throw_error(
            "collapse_edge({}->{}) disagrees with the brute-force reference.\nexpected:\n{}\ngot:"
            "\n{}",
            bf_removed,
            bf_kept,
            brute_force_expected,
            debug_canonical_form());
    }
#endif

    start_protect_attributes();

    if (!collapse_edge_after(return_t) || !invariants(new_tris)) {
        // if call back check failed roll back
        // restore the changes for connected triangles and vertices
        // resotre the version-number
        // removed restore to false

        // for (auto rollback : old_tris) {
        //     size_t fid = rollback.first;
        //     m_tri_connectivity[fid] = rollback.second;
        // }
        // for (auto rollback : old_vertices) {
        //     size_t vid = rollback.first;
        //     m_vertex_connectivity[vid] = rollback.second;
        // }
        // m_vertex_connectivity[new_vid].m_conn_tris.clear();
        // m_vertex_connectivity[new_vid].m_is_removed = true;
        // for (auto vid_fid : same_edge_vid_fid) {
        //     size_t vid = vid_fid.first;
        //     size_t fid = vid_fid.second;
        //     m_vertex_connectivity[vid].m_conn_tris.push_back(fid);
        //     std::sort(
        //         m_vertex_connectivity[vid].m_conn_tris.begin(),
        //         m_vertex_connectivity[vid].m_conn_tris.end());
        // }
        // for (size_t fid : n12_intersect_fids) {
        //     m_tri_connectivity[fid].m_is_removed = false;
        // }
        collapse_edge_rollback(
            new_vid,
            old_tris,
            old_vertices,
            same_edge_vid_fid,
            n12_intersect_fids);

        rollback_protected_attributes();
        return false;
    }
    release_protect_attributes();
    return true;
}

void TriMesh::collapse_edge_rollback(
    size_t& new_vid,
    std::vector<std::pair<size_t, TriangleConnectivity>>& old_tris,
    std::vector<std::pair<size_t, VertexConnectivity>>& old_vertices,
    std::vector<std::pair<size_t, size_t>>& same_edge_vid_fid,
    std::vector<size_t>& n12_intersect_fids)
{
    for (auto rollback : old_tris) {
        size_t fid = rollback.first;
        m_tri_connectivity[fid] = rollback.second;
    }
    for (auto rollback : old_vertices) {
        size_t vid = rollback.first;
        m_vertex_connectivity[vid] = rollback.second;
    }

    // CHANGE: no new vertex
    // m_vertex_connectivity[new_vid].m_conn_tris.clear();
    // m_vertex_connectivity[new_vid].m_is_removed = true;


    for (auto vid_fid : same_edge_vid_fid) {
        size_t vid = vid_fid.first;
        size_t fid = vid_fid.second;
        m_vertex_connectivity[vid].m_conn_tris.push_back(fid);
        std::sort(
            m_vertex_connectivity[vid].m_conn_tris.begin(),
            m_vertex_connectivity[vid].m_conn_tris.end());
    }
    for (size_t fid : n12_intersect_fids) {
        m_tri_connectivity[fid].m_is_removed = false;
    }

    // old_tris restores each face's own cycles, but the forward pass also rewrote the
    // cycles of faces outside that set -- the far side of every edge whose fan it touched.
    // Recomputing over the restored connectivity puts all of them back; the cycles are a
    // function of the connectivity, so this is exact rather than approximate.
    {
        thread_local std::vector<std::pair<size_t, size_t>> edges;
        thread_local std::vector<size_t> verts;
        edges.clear();
        verts.clear();
        for (const auto& [fid, conn] : old_tris) {
            for (int j = 0; j < 3; ++j) {
                verts.push_back(conn.m_indices[j]);
                const size_t a = conn.m_indices[(j + 1) % 3];
                const size_t b = conn.m_indices[(j + 2) % 3];
                if (a != b) edges.emplace_back(std::min(a, b), std::max(a, b));
            }
        }
        rebuild_cycles_for(edges, verts);
    }

    rollback_protected_attributes();
}


void TriMesh::collapse_edge_conn(
    const Tuple& loc0,
    std::vector<Tuple>& new_tris,
    Tuple& return_t,
    size_t& new_vid,
    std::vector<std::pair<size_t, TriangleConnectivity>>& old_tris,
    std::vector<std::pair<size_t, VertexConnectivity>>& old_vertices,
    std::vector<std::pair<size_t, size_t>>& same_edge_vid_fid,
    std::vector<size_t>& n12_intersect_fids)
{
    // get fid for the return tuple
    // take the face that shares the same vertex the loc0 tuple is pointing to
    // or if that face doesn't exit
    // take the face that shares the same vertex of loc0
    const SmartTuple smart_loc0(*this, loc0);
    const SmartTuple opp_edge = smart_loc0.switch_vertex().switch_edge();
    const auto opp_edge_opp_faces = opp_edge.tuple().switch_faces(*this);
    size_t new_fid = -1;
    if (!opp_edge_opp_faces.empty()) {
        new_fid = opp_edge_opp_faces[0].fid(*this);
    } else {
        const SmartTuple inc_edge = smart_loc0.switch_edge();
        const auto inc_edge_opp_faces = inc_edge.tuple().switch_faces(*this);
        if (!inc_edge_opp_faces.empty()) {
            new_fid = inc_edge_opp_faces[0].fid(*this);
        }
        // Otherwise both of loc0's other edges are on the boundary, so no face of the
        // collapsed edge's star survives to carry the return tuple. That is reachable
        // whenever the collapsed triangle is only attached to the rest of the mesh through
        // its vertices -- one wing of a bowtie, say. The link condition used to make it
        // unreachable, which is why this arm used to be an assert. Resolved below, once the
        // survivor's face list is known.
    }

    // get the vids
    const size_t vid1 = loc0.vid(*this);
    const size_t vid2 = switch_vertex(loc0).vid(*this);

    // record the vids that will be erased for roll backs on failure
    old_vertices.resize(2);
    old_vertices[0] = std::make_pair(vid1, m_vertex_connectivity[vid1]);
    old_vertices[1] = std::make_pair(vid2, m_vertex_connectivity[vid2]);

    // get the fids
    const auto n1_fids = m_vertex_connectivity[vid1].m_conn_tris;
    const auto n2_fids = m_vertex_connectivity[vid2].m_conn_tris;

    // get the fids that will be modified
    n12_intersect_fids = set_intersection(n1_fids, n2_fids);
#ifndef NDEBUG
    {
        // check if the triangles intersection is the one adjcent to the edge
        assert(vector_contains(n12_intersect_fids, loc0.fid(*this)));
        for (const Tuple& loc1 : loc0.switch_faces(*this)) {
            assert(vector_contains(n12_intersect_fids, loc1.fid(*this)));
        }
        //"faces at the edge is not correct"
    }
#endif // !NDEBUG

    // now mark the vertices as removed so the assertion for tuple validity in switch operations
    // won't fail

    // CHANGE: preserve v2 now, do not generate new vertex for collapse
    m_vertex_connectivity[vid1].m_is_removed = true;
    // m_vertex_connectivity[vid2].m_is_removed = true;
    for (size_t fid : n12_intersect_fids) {
        m_tri_connectivity[fid].m_is_removed = true;
    }

    std::vector<size_t> n12_union_fids;
    std::set_union(
        n1_fids.begin(),
        n1_fids.end(),
        n2_fids.begin(),
        n2_fids.end(),
        std::back_inserter(n12_union_fids));

    // record the fids that will be modified/erased for roll back on failure
    vector_unique(n12_union_fids);
    old_tris.resize(n12_union_fids.size());

    for (int i = 0; i < old_tris.size(); i++) {
        size_t fid = n12_union_fids[i];
        old_tris[i] = std::make_pair(fid, m_tri_connectivity[fid]);
        m_tri_connectivity[fid].hash++;
    }
    // modify the triangles
    // the m_conn_tris needs to be sorted

    // CHANGE: new_vid now is v2
    // new_vid = get_next_empty_slot_v();
    new_vid = vid2;

    for (size_t fid : n1_fids) {
        if (m_tri_connectivity[fid].m_is_removed)
            continue;
        else {
            int j = m_tri_connectivity[fid].find(vid1);

            // CHANGE: new_vid now is v2
            // m_tri_connectivity[fid].m_indices[j] = new_vid;
            m_tri_connectivity[fid].m_indices[j] = vid2;
        }
    }
    // CHANGE: no need to update
    // for (size_t fid : n2_fids) {
    //     if (m_tri_connectivity[fid].m_is_removed)
    //         continue;
    //     else {
    //         int j = m_tri_connectivity[fid].find(vid2);
    //         m_tri_connectivity[fid].m_indices[j] = new_vid;
    //     }
    // }

    // now work on vids
    // add in the new vertex

    // CHANGE: change vf connectivity f v2 instead of adding new

    // add preserved v1 faces to v2
    // clear v2 conn first
    m_vertex_connectivity[vid2].m_conn_tris.clear();
    for (size_t fid : n12_union_fids) {
        if (m_tri_connectivity[fid].m_is_removed) {
            continue;
        } else {
            m_vertex_connectivity[vid2].m_conn_tris.push_back(fid);
        }
    }

    // this is sorting
    vector_unique(m_vertex_connectivity[vid2].m_conn_tris);

    // for (size_t fid : n12_union_fids) {
    //     if (m_tri_connectivity[fid].m_is_removed)
    //         continue;
    //     else
    //         m_vertex_connectivity[new_vid].m_conn_tris.push_back(fid);
    // }
    // // This is sorting too, and it is important to sort
    // vector_unique(m_vertex_connectivity[new_vid].m_conn_tris);

    // remove the erased fids from the vertices' (the one of the triangles that is not the end
    // points of the edge) connectivity list

    for (size_t fid : n12_intersect_fids) {
        auto f_vids = m_tri_connectivity[fid].m_indices;
        for (size_t f_vid : f_vids) {
            if (f_vid != vid1 && f_vid != vid2) {
                same_edge_vid_fid.emplace_back(f_vid, fid);
                assert(vector_contains(m_vertex_connectivity[f_vid].m_conn_tris, fid));
                vector_erase(m_vertex_connectivity[f_vid].m_conn_tris, fid);
            }
        }
    }

    // Merge duplicate triangles.
    //
    // Two faces end up with the same vertex triple exactly when some edge {x,y} was in the
    // link of both endpoints -- which is what the second half of check_link_condition()
    // forbids. With the link condition off that can happen, and a simplicial complex cannot
    // hold the same triangle twice: tuple_from_vids() would throw and every fan-based
    // routine would have to tolerate the pair. Keep the smallest fid of each group and drop
    // the rest. The dropped faces' own attributes go with them.
    {
        std::map<std::array<size_t, 3>, size_t> first_with_key;
        std::vector<size_t> merged_away;
        // m_conn_tris is sorted, so the first face seen for a key is the smallest fid.
        for (const size_t fid : m_vertex_connectivity[vid2].m_conn_tris) {
            std::array<size_t, 3> key = m_tri_connectivity[fid].m_indices;
            std::sort(key.begin(), key.end());
            if (!first_with_key.try_emplace(key, fid).second) {
                merged_away.push_back(fid);
            }
        }
        for (const size_t fid : merged_away) {
            m_tri_connectivity[fid].m_is_removed = true;
            for (const size_t f_vid : m_tri_connectivity[fid].m_indices) {
                if (f_vid == vid1 || f_vid == vid2) continue;
                // Recorded the same way as the faces at the collapsed edge, so the existing
                // rollback path restores them without knowing why they went away.
                same_edge_vid_fid.emplace_back(f_vid, fid);
                vector_erase(m_vertex_connectivity[f_vid].m_conn_tris, fid);
            }
            vector_erase(m_vertex_connectivity[vid2].m_conn_tris, fid);
        }
    }

    // Retire vertices left with no incident face.
    //
    // A vertex joined to the collapsed edge by a single triangle loses it here; if it has
    // nothing else it is live but faceless, which no navigation can express. Capturing it
    // in old_vertices first means the existing rollback un-removes it: the restore puts
    // back m_is_removed == false, and the same_edge_vid_fid replay that follows refills its
    // face list.
    {
        std::set<size_t> candidates;
        for (const auto& [v, f] : same_edge_vid_fid) candidates.insert(v);
        for (const size_t v : candidates) {
            if (!m_vertex_connectivity[v].m_is_removed &&
                m_vertex_connectivity[v].m_conn_tris.empty()) {
                old_vertices.emplace_back(v, m_vertex_connectivity[v]);
                m_vertex_connectivity[v].m_is_removed = true;
            }
        }
    }

    // Rebuild the cycles over everything the collapse touched.
    //
    // Every changed face is in old_tris, which holds their vertices as they were *before*
    // the relabel; renaming vid1 to vid2 there gives exactly the edges and vertices whose
    // fans can have moved. Nothing outside that set is affected, and every vertex is
    // visited once -- including one whose only face at the collapsed edge was removed,
    // which none of the surviving faces would lead to.
    {
        thread_local std::vector<std::pair<size_t, size_t>> edges;
        thread_local std::vector<size_t> verts;
        edges.clear();
        verts.clear();
        for (const auto& [fid, old_conn] : old_tris) {
            std::array<size_t, 3> v = old_conn.m_indices;
            for (size_t& x : v) {
                if (x == vid1) x = vid2;
            }
            for (int j = 0; j < 3; ++j) {
                verts.push_back(v[j]);
                const size_t a = v[(j + 1) % 3];
                const size_t b = v[(j + 2) % 3];
                // the face carrying both endpoints collapses to a degenerate edge
                if (a != b) edges.emplace_back(std::min(a, b), std::max(a, b));
            }
        }
        rebuild_cycles_for(edges, verts);
    }

    // ? ? tuples changes. this needs to be done before post check since checked are done on tuples
    // update the old tuple version number
    // create an edge tuple for each changed edge
    // call back check will be done on this vector of tuples

    assert(m_vertex_connectivity[new_vid].m_conn_tris.size() != 0);

    // CHANGE: new_vid to vid2,


    const size_t gfid = m_vertex_connectivity[new_vid].m_conn_tris[0];
    int j = m_tri_connectivity[gfid].find(new_vid);
    auto new_t = Tuple(new_vid, (j + 2) % 3, gfid, *this);
    // new_fid was picked before the mutation. It may have been left unset because nothing
    // in the star survives, or it may since have been merged away as a duplicate; either
    // way the survivor's own first face is a valid home for the return tuple.
    if (new_fid == size_t(-1) || m_tri_connectivity[new_fid].m_is_removed) {
        new_fid = gfid;
    }
    int j_ret = m_tri_connectivity[new_fid].find(new_vid);
    return_t = Tuple(new_vid, (j_ret + 2) % 3, new_fid, *this);
    assert(new_t.is_valid(*this));
    new_tris = get_one_ring_tris_for_vertex(new_t);
}


bool TriMesh::swap_edge(const Tuple& t, std::vector<Tuple>& new_tris)
{
    if (!swap_edge_before(t)) {
        return false;
    }

    // get the vids
    size_t vid1 = t.vid(*this);
    size_t vid2 = t.switch_vertex(*this).vid(*this);

    const auto t_opps = t.switch_faces(*this);
    if (t_opps.size() != 1) {
        // should be already checked in swap_edge_before
        return false; // can't sawp on boundary or non-manifold edge
    }

    Tuple tmp_tuple;
    tmp_tuple = t_opps[0];
    assert(tmp_tuple.is_valid(*this));
    tmp_tuple = tmp_tuple.switch_edge(*this);
    size_t vid3 = tmp_tuple.switch_vertex(*this).vid(*this);
    auto tmp_tuple2 = t.switch_edge(*this);
    assert(tmp_tuple2.is_valid(*this));
    size_t vid4 = tmp_tuple2.switch_vertex(*this).vid(*this);
    // record the vids that will be changed for roll backs on failure
    // namely the 4 vertices of the 2 triangles
    std::vector<std::pair<size_t, VertexConnectivity>> old_vertices(4);
    old_vertices[0] = std::make_pair(vid1, m_vertex_connectivity[vid1]);
    old_vertices[1] = std::make_pair(vid2, m_vertex_connectivity[vid2]);
    old_vertices[2] = std::make_pair(vid3, m_vertex_connectivity[vid3]);
    old_vertices[3] = std::make_pair(vid4, m_vertex_connectivity[vid4]);

    // check if the triangles intersection is the one adjcent to the edge
    size_t test_fid1 = t.fid(*this);
    size_t test_fid2 = t_opps[0].fid(*this);

#ifdef WMTK_DEBUG_BRUTE_FORCE_OPS
    if (!check_mesh_connectivity_validity()) {
        log_and_throw_error("swap_edge: connectivity was already invalid on entry");
    }
    const std::string brute_force_expected = debug_reference_swap(vid1, vid2, test_fid1, test_fid2);
#endif
    // record the fids that will be changed for roll backs on failure
    std::vector<std::pair<size_t, TriangleConnectivity>> old_tris(2);
    old_tris[0] = std::make_pair(test_fid1, m_tri_connectivity[test_fid1]);
    old_tris[1] = std::make_pair(test_fid2, m_tri_connectivity[test_fid2]);

    // first work on triangles, there are only 2
    int j = m_tri_connectivity[test_fid1].find(vid2);
    m_tri_connectivity[test_fid1].m_indices[j] = vid3;
    m_tri_connectivity[test_fid1].hash++;

    j = m_tri_connectivity[test_fid2].find(vid1);
    m_tri_connectivity[test_fid2].m_indices[j] = vid4;
    m_tri_connectivity[test_fid2].hash++;

    // then work on the vertices
    vector_erase(m_vertex_connectivity[vid1].m_conn_tris, test_fid2);
    vector_erase(m_vertex_connectivity[vid2].m_conn_tris, test_fid1);
    m_vertex_connectivity[vid3].m_conn_tris.push_back(test_fid1);
    vector_unique(m_vertex_connectivity[vid3].m_conn_tris);
    m_vertex_connectivity[vid4].m_conn_tris.push_back(test_fid2);
    vector_unique(m_vertex_connectivity[vid4].m_conn_tris);

    // (vid1,vid2) is gone and (vid3,vid4) is new; the four surviving rim edges each changed
    // which face carries them.
    const std::vector<size_t> swapped = {test_fid1, test_fid2};
    rebuild_cycles_around(swapped);

#ifdef WMTK_DEBUG_BRUTE_FORCE_OPS
    if (debug_canonical_form() != brute_force_expected) {
        log_and_throw_error(
            "swap_edge({},{}) disagrees with the brute-force reference.\nexpected:\n{}\ngot:\n{}",
            vid1,
            vid2,
            brute_force_expected,
            debug_canonical_form());
    }
#endif

    // change the tuple to the new edge tuple
    auto new_t = tuple_from_edge(vid4, vid3, test_fid2);

    assert(new_t.switch_vertex(*this).vid(*this) != vid1);
    assert(new_t.switch_vertex(*this).vid(*this) != vid2);
    assert(new_t.is_valid(*this));
    new_tris = {new_t, new_t.switch_faces(*this)[0]};
    start_protect_attributes();
    if (!swap_edge_after(new_t) || !invariants(new_tris)) {
        // restore the vertex and faces
        for (const auto& old_v : old_vertices) {
            m_vertex_connectivity[old_v.first] = old_v.second;
        }
        for (const auto& old_tri : old_tris) {
            m_tri_connectivity[old_tri.first] = old_tri.second;
        }
        rebuild_cycles_around(swapped);
        rollback_protected_attributes();

        return false;
    }
    release_protect_attributes();
    return true;
}

bool TriMesh::smooth_vertex(const Tuple& loc0)
{
    if (!smooth_before(loc0)) return false;

#ifdef WMTK_DEBUG_BRUTE_FORCE_OPS
    // Smoothing moves a vertex without touching connectivity, so the reference is simply
    // that nothing about the mesh changed. Worth checking: a subclass whose smooth_after
    // reached into the topology would be caught here rather than much later.
    if (!check_mesh_connectivity_validity()) {
        log_and_throw_error("smooth_vertex: connectivity was already invalid on entry");
    }
    const std::string brute_force_expected = debug_canonical_form();
#endif

    start_protect_attributes();
    if (!smooth_after(loc0) || !invariants(get_one_ring_tris_for_vertex(loc0))) {
        rollback_protected_attributes();
        return false;
    }
    release_protect_attributes();

#ifdef WMTK_DEBUG_BRUTE_FORCE_OPS
    if (debug_canonical_form() != brute_force_expected) {
        log_and_throw_error(
            "smooth_vertex({}) changed the connectivity, which it must not.\nexpected:\n{}\ngot:"
            "\n{}",
            loc0.vid(*this),
            brute_force_expected,
            debug_canonical_form());
    }
#endif
    return true;
}

bool TriMesh::split_face(const Tuple& t, std::vector<Tuple>& new_tris)
{
    if (!split_face_before(t)) {
        return false;
    }
    if (!t.is_valid(*this)) {
        return false;
    }

    // get local eid for return tuple construction
    const size_t local_eid = t.local_eid(*this);

    /**
     *
     *
     *         v2
     *         /|\
     *        / | \
     *       /  |  \
     *      /f1 ^ f0\
     *     /  /   \  \
     *    //   f2    \\
     *  v0 ----------- v1
     *
     */

    const size_t fid = t.fid(*this);

    std::array<size_t, 3> vid;
    vid[0] = t.vid(*this);
    vid[1] = t.switch_vertex(*this).vid(*this);
    vid[2] = t.switch_edge(*this).switch_vertex(*this).vid(*this);

    // record the vids that will be modified for roll backs on failure
    std::array<std::pair<size_t, VertexConnectivity>, 3> old_vertices;
    old_vertices[0] = {vid[0], m_vertex_connectivity[vid[0]]};
    old_vertices[1] = {vid[1], m_vertex_connectivity[vid[1]]};
    old_vertices[2] = {vid[2], m_vertex_connectivity[vid[2]]};
    std::pair<size_t, TriangleConnectivity> old_tri;
    old_tri = std::make_pair(fid, m_tri_connectivity[fid]);

    const auto conn_tris = [this, &vid](size_t i) -> std::vector<size_t>& {
        return m_vertex_connectivity[vid[i]].m_conn_tris;
    };

#ifdef WMTK_DEBUG_BRUTE_FORCE_OPS
    if (!check_mesh_connectivity_validity()) {
        log_and_throw_error("split_face: connectivity was already invalid on entry");
    }
    // Before the slots are reserved, for the same reason as split_edge.
    const size_t bf_tri_cap = tri_capacity();
#endif

    // update vertex connectivity
    const size_t new_vid = get_next_empty_slot_v();
    const size_t new_fid1 = get_next_empty_slot_t();
    const size_t new_fid2 = get_next_empty_slot_t();

    // abort before mutating if we ran out of preallocated slots; mark any reserved
    // slots removed so they don't become live-but-empty phantoms.
    {
        constexpr size_t INVALID_SLOT = static_cast<size_t>(-1);
        if (new_vid == INVALID_SLOT || new_fid1 == INVALID_SLOT || new_fid2 == INVALID_SLOT) {
            if (new_vid != INVALID_SLOT) m_vertex_connectivity[new_vid].m_is_removed = true;
            if (new_fid1 != INVALID_SLOT) m_tri_connectivity[new_fid1].m_is_removed = true;
            if (new_fid2 != INVALID_SLOT) m_tri_connectivity[new_fid2].m_is_removed = true;
            return false;
        }
    }

#ifdef WMTK_DEBUG_BRUTE_FORCE_OPS
    // After the slots are known, before any of the connectivity moves.
    const std::string brute_force_expected =
        debug_reference_split_face(fid, vid[0], vid[1], vid[2], new_vid, bf_tri_cap);
#endif

    vector_erase(conn_tris(0), fid);
    conn_tris(0).emplace_back(new_fid1);
    conn_tris(0).emplace_back(new_fid2);
    std::sort(conn_tris(0).begin(), conn_tris(0).end());
    conn_tris(1).emplace_back(new_fid2);
    std::sort(conn_tris(1).begin(), conn_tris(1).end());
    conn_tris(2).emplace_back(new_fid1);
    std::sort(conn_tris(2).begin(), conn_tris(2).end());

    m_vertex_connectivity[new_vid].m_conn_tris.reserve(3);
    m_vertex_connectivity[new_vid].m_conn_tris.emplace_back(fid);
    m_vertex_connectivity[new_vid].m_conn_tris.emplace_back(new_fid1);
    m_vertex_connectivity[new_vid].m_conn_tris.emplace_back(new_fid2);
    std::sort(
        m_vertex_connectivity[new_vid].m_conn_tris.begin(),
        m_vertex_connectivity[new_vid].m_conn_tris.end());

    // now the triangles
    // need to update the hash of fid
    const size_t i = m_tri_connectivity[fid].find(vid[0]);
    const size_t j = m_tri_connectivity[fid].find(vid[1]);
    const size_t k = m_tri_connectivity[fid].find(vid[2]);
    m_tri_connectivity[fid].m_indices[i] = new_vid;
    m_tri_connectivity[fid].hash++;
    // new_fid1/2 m_indices in same order
    m_tri_connectivity[new_fid1].m_indices[i] = vid[0];
    m_tri_connectivity[new_fid1].m_indices[j] = new_vid;
    m_tri_connectivity[new_fid1].m_indices[k] = vid[2];
    m_tri_connectivity[new_fid2].m_indices[i] = vid[0];
    m_tri_connectivity[new_fid2].m_indices[j] = vid[1];
    m_tri_connectivity[new_fid2].m_indices[k] = new_vid;

    // The three outer edges keep their fans but change which face carries them, and the
    // three spokes to new_vid are new.
    const std::vector<size_t> split_fids = {fid, new_fid1, new_fid2};
    rebuild_cycles_around(split_fids);

#ifdef WMTK_DEBUG_BRUTE_FORCE_OPS
    if (debug_canonical_form() != brute_force_expected) {
        log_and_throw_error(
            "split_face({}) disagrees with the brute-force reference.\nexpected:\n{}\ngot:\n{}",
            fid,
            brute_force_expected,
            debug_canonical_form());
    }
#endif

    // make the new tuple
    Tuple new_vertex_tuple(new_vid, (k + 2) % 3, new_fid2, *this);
    Tuple return_tuple(vid[0], local_eid, new_fid2, *this);
    assert(return_tuple.is_valid(*this));

    new_tris = get_one_ring_tris_for_vertex(new_vertex_tuple);
    start_protect_attributes();

    // roll back if not successful
    if (!split_face_after(return_tuple) || !invariants(new_tris)) {
        // rollback topo
        // restore old v, t
        for (const auto& old_v : old_vertices) {
            m_vertex_connectivity[old_v.first] = old_v.second;
        }
        m_tri_connectivity[old_tri.first] = old_tri.second;

        // erase new_vid new_fids
        m_vertex_connectivity[new_vid].m_conn_tris.clear();
        m_vertex_connectivity[new_vid].m_is_removed = true;
        m_tri_connectivity[new_fid1].m_is_removed = true;
        m_tri_connectivity[new_fid2].m_is_removed = true;
        rebuild_cycles_around(split_fids);
        rollback_protected_attributes();
        return false;
    }
    release_protect_attributes();
    return true;
}

void TriMesh::consolidate_mesh()
{
    auto v_cnt = 0;
    std::vector<size_t> map_v_ids(vert_capacity(), -1);
    for (auto i = 0; i < vert_capacity(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        map_v_ids[i] = v_cnt;
        v_cnt++;
    }
    auto t_cnt = 0;
    std::vector<size_t> map_t_ids(tri_capacity(), -1);
    for (auto i = 0; i < tri_capacity(); i++) {
        if (m_tri_connectivity[i].m_is_removed) continue;
        map_t_ids[i] = t_cnt;
        t_cnt++;
    }
    v_cnt = 0;
    for (auto i = 0; i < vert_capacity(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        if (v_cnt != i) {
            assert(v_cnt < i);
            m_vertex_connectivity[v_cnt] = m_vertex_connectivity[i];
            if (p_vertex_attrs) p_vertex_attrs->move(i, v_cnt);
        }
        for (size_t& t_id : m_vertex_connectivity[v_cnt].m_conn_tris) t_id = map_t_ids[t_id];
        v_cnt++;
    }
    t_cnt = 0;
    for (int i = 0; i < tri_capacity(); i++) {
        if (m_tri_connectivity[i].m_is_removed) continue;

        if (t_cnt != i) {
            assert(t_cnt < i);
            m_tri_connectivity[t_cnt] = m_tri_connectivity[i];
            m_tri_connectivity[t_cnt].hash = 0;
            if (p_face_attrs) p_face_attrs->move(i, t_cnt);

            for (auto j = 0; j < 3; j++) {
                if (p_edge_attrs) p_edge_attrs->move(i * 3 + j, t_cnt * 3 + j);
            }
        }
        for (size_t& v_id : m_tri_connectivity[t_cnt].m_indices) v_id = map_v_ids[v_id];
        t_cnt++;
    }

    current_vert_size = v_cnt;
    current_tri_size = t_cnt;

    // Re-establish spare capacity for the next round of operations.
    const size_t vcap = reserved_capacity(v_cnt);
    const size_t tcap = reserved_capacity(t_cnt);

    m_vertex_connectivity.resize(vcap);
    m_vertex_connectivity.shrink_to_fit();
    m_tri_connectivity.resize(tcap);
    m_tri_connectivity.shrink_to_fit();

    // Every fid moved, so the stored successors are stale. Cheaper to redo them all than to
    // remap in place, and this runs once per pass.
    rebuild_all_cycles();

    resize_mutex(vcap);

    // Resize user class attributes to the preallocated capacity
    if (p_vertex_attrs) p_vertex_attrs->resize(vcap);
    if (p_edge_attrs) p_edge_attrs->resize(tcap * 3);
    if (p_face_attrs) p_face_attrs->resize(tcap);

    // assert(check_edge_manifold());
    assert(check_mesh_connectivity_validity());
}


std::vector<size_t> TriMesh::get_one_ring_vids_for_vertex_duplicate(const size_t& vid) const
{
    std::vector<size_t> one_ring;
    get_one_ring_vids_for_vertex_duplicate(vid, one_ring);
    return one_ring;
}

void wmtk::TriMesh::get_one_ring_vids_for_vertex_duplicate(
    const size_t& vid,
    std::vector<size_t>& one_ring) const
{
    const auto& conn_tri = m_vertex_connectivity[vid].m_conn_tris;

    one_ring.clear();
    one_ring.reserve(conn_tri.size() * 4);
    for (size_t tri : conn_tri) {
        for (size_t j : m_tri_connectivity[tri].m_indices) {
            one_ring.push_back(j);
        }
    }
}

std::vector<size_t> TriMesh::get_incident_fids_for_edge(const Tuple& t) const
{
    return get_incident_fids_for_edge(t.vid(*this), t.switch_vertex(*this).vid(*this));
}

std::vector<size_t> TriMesh::get_incident_fids_for_edge(const size_t vid0, const size_t vid1) const
{
    const auto& v0 = get_one_ring_fids_for_vertex(vid0);
    const auto& v1 = get_one_ring_fids_for_vertex(vid1);
    return set_intersection(v0, v1);
}

std::vector<TriMesh::Tuple> TriMesh::get_one_ring_tris_for_vertex(const TriMesh::Tuple& t) const
{
    std::vector<TriMesh::Tuple> one_ring;
    size_t vid = t.vid(*this);
    auto& conn_tri = m_vertex_connectivity[vid].m_conn_tris;
    one_ring.reserve(conn_tri.size());
    for (size_t tri : conn_tri) {
        int j = m_tri_connectivity[tri].find(vid);
        one_ring.emplace_back(vid, (j + 2) % 3, tri, *this);
        assert(one_ring[one_ring.size() - 1].is_valid(*this));
    }

    return one_ring;
}

const std::vector<size_t>& TriMesh::get_one_ring_fids_for_vertex(const Tuple& t) const
{
    return get_one_ring_fids_for_vertex(t.vid(*this));
}

const std::vector<size_t>& TriMesh::get_one_ring_fids_for_vertex(const size_t vid) const
{
    return m_vertex_connectivity[vid].m_conn_tris;
}

std::vector<wmtk::TriMesh::Tuple> TriMesh::get_one_ring_edges_for_vertex(
    const wmtk::TriMesh::Tuple& t) const
{
    std::vector<Tuple> one_ring_edges;
    std::vector<size_t> one_ring_vertices;
    size_t vid = t.vid(*this);
    auto one_ring_tris = get_one_ring_tris_for_vertex(t);
    for (Tuple tri : one_ring_tris) {
        // find the vertex
        while (tri.vid(*this) != vid) {
            tri = tri.switch_vertex(*this).switch_edge(*this);
        }

        // push first edge if not there
        if (!vector_contains(one_ring_vertices, tri.switch_vertex(*this).vid(*this))) {
            one_ring_vertices.push_back(tri.switch_vertex(*this).vid(*this));
            one_ring_edges.push_back(tri.switch_vertex(*this));
        }

        // push second edge if not there
        tri = tri.switch_edge(*this);
        if (!vector_contains(one_ring_vertices, tri.switch_vertex(*this).vid(*this))) {
            one_ring_vertices.push_back(tri.switch_vertex(*this).vid(*this));
            one_ring_edges.push_back(tri.switch_vertex(*this));
        }
    }

    assert(one_ring_vertices.size() == one_ring_edges.size());

    /**
     * The code below is a faster implementation but it did not give the exact same
     * result for QSLIM. Leaving it commented out for now.
     */

    // const size_t vid = t.vid(*this);
    //
    // std::map<size_t, Tuple> vid_tup;
    // for (const size_t fid : m_vertex_connectivity[vid].m_conn_tris) {
    //     const auto& vids = m_tri_connectivity[fid].m_indices;
    //     const int loc_v0 = m_tri_connectivity[fid].find(vid);
    //     assert(loc_v0 >= 0);
    //     const int loc_v1 = (loc_v0 + 1) % 3;
    //     const size_t v1 = vids[loc_v1];
    //     if (vid_tup.count(v1) == 0) {
    //         const int e1 = 3 - (loc_v0 + loc_v1);
    //         vid_tup[v1] = Tuple(v1, e1, fid, *this);
    //     }
    //
    //     const int loc_v2 = (loc_v0 + 2) % 3;
    //     const size_t v2 = vids[loc_v2];
    //     if (vid_tup.count(v2) == 0) {
    //         const int e2 = 3 - (loc_v0 + loc_v1);
    //         vid_tup[v2] = Tuple(v2, e2, fid, *this);
    //     }
    // }
    //
    // std::vector<Tuple> one_ring_edges;
    // one_ring_edges.reserve(vid_tup.size());
    // for (const auto& [v, edge_tuple] : vid_tup) {
    //     one_ring_edges.emplace_back(edge_tuple);
    // }

    return one_ring_edges;
}

std::vector<wmtk::TriMesh::Tuple> wmtk::TriMesh::get_one_ring_edges_for_vertex(
    const size_t vid) const
{
    const Tuple t = tuple_from_vertex(vid);
    return get_one_ring_edges_for_vertex(t);
}

std::array<wmtk::TriMesh::Tuple, 3> TriMesh::oriented_tri_vertices(
    const wmtk::TriMesh::Tuple& t) const
{
    std::array<TriMesh::Tuple, 3> incident_verts;
    size_t fid = t.fid(*this);
    auto indices = m_tri_connectivity[fid].m_indices;

    incident_verts[0] = Tuple(indices[0], 2, fid, *this);
    incident_verts[1] = Tuple(indices[1], 0, fid, *this);
    incident_verts[2] = Tuple(indices[2], 1, fid, *this);
    return incident_verts;
}

std::array<size_t, 3> TriMesh::oriented_tri_vids(const Tuple& t) const
{
    size_t fid = t.fid(*this);
    return oriented_tri_vids(fid);
}

std::array<size_t, 3> TriMesh::oriented_tri_vids(const size_t fid) const
{
    std::array<size_t, 3> incident_verts;
    auto indices = m_tri_connectivity[fid].m_indices;

    incident_verts[0] = indices[0];
    incident_verts[1] = indices[1];
    incident_verts[2] = indices[2];

    return incident_verts;
}

std::array<TriMesh::Tuple, 2> TriMesh::get_edge_vertices(const Tuple& t) const
{
    return std::array<TriMesh::Tuple, 2>{{t, t.switch_vertex(*this)}};
}

std::array<size_t, 2> TriMesh::get_edge_vids(const Tuple& t) const
{
    return std::array<size_t, 2>{{t.vid(*this), t.switch_vertex(*this).vid(*this)}};
}

std::tuple<TriMesh::Tuple, size_t> TriMesh::tuple_from_edge(const std::array<size_t, 2>& vids) const
{
    const std::vector<size_t>& fids = m_vertex_connectivity[vids[0]].m_conn_tris;

    // find face that contains both vertices
    size_t local_eid = std::numeric_limits<size_t>::max();
    size_t fid = std::numeric_limits<size_t>::max();
    for (const size_t f : fids) {
        const auto& f_vids = m_tri_connectivity[f].m_indices;

        int local_v0 = -1;
        int local_v1 = -1;
        for (size_t i = 0; i < f_vids.size(); ++i) {
            if (f_vids[i] == vids[0]) {
                local_v0 = (int)i;
            } else if (f_vids[i] == vids[1]) {
                local_v1 = (int)i;
            }
        }
        if (local_v1 == -1) {
            continue;
        }
        assert(local_v0 != -1);
        local_eid = 3 - (local_v0 + local_v1);
        fid = f;
        break;
    }
    assert(local_eid != std::numeric_limits<size_t>::max());

    const Tuple edge(vids[0], local_eid, fid, *this);
    const size_t eid = 3 * fid + local_eid;
    return std::make_tuple(edge, eid);
}


void TriMesh::init(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris)
{
    // Preallocate spare capacity; only [0, live) is filled, the rest is headroom
    // that operations consume (and fail past). See reserved_capacity().
    const size_t vcap = reserved_capacity(n_vertices);
    const size_t tcap = reserved_capacity(tris.size());
    m_vertex_connectivity.resize(vcap);
    m_tri_connectivity.resize(tcap);
    size_t hash_cnt = 0;
    for (int i = 0; i < tris.size(); i++) {
        m_tri_connectivity[i].m_indices = tris[i];

        m_tri_connectivity[i].hash = hash_cnt;
        for (int j = 0; j < 3; j++) {
            m_vertex_connectivity[tris[i][j]].m_conn_tris.push_back(i);
        }
    }
    current_vert_size = (long)n_vertices;
    current_tri_size = (long)tris.size();

    rebuild_all_cycles();

    resize_mutex(vcap);

    // Resize user class attributes to the preallocated capacity
    if (p_vertex_attrs) p_vertex_attrs->resize(vcap);
    if (p_edge_attrs) p_edge_attrs->resize(tcap * 3);
    if (p_face_attrs) p_face_attrs->resize(tcap);
}

void wmtk::TriMesh::init(const MatrixXi& F)
{
    size_t n_vertices = F.maxCoeff() + 1;

    std::vector<std::array<size_t, 3>> tris;
    tris.resize(F.rows());

    for (int i = 0; i < F.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            tris[i][j] = F(i, j);
        }
    }

    TriMesh::init(n_vertices, tris);
}

std::vector<TriMesh::Tuple> TriMesh::get_vertices() const
{
    const size_t n_vertices = vert_capacity();
    std::vector<Tuple> all_vertices_tuples;
    all_vertices_tuples.reserve(n_vertices);

    for (size_t i = 0; i < n_vertices; i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;

        const std::vector<size_t>& v_conn_fids = m_vertex_connectivity[i].m_conn_tris;
        size_t fid = *min_element(v_conn_fids.begin(), v_conn_fids.end());

        // get the 3 vid
        const std::array<size_t, 3> f_conn_verts = m_tri_connectivity[fid].m_indices;
        assert(i == f_conn_verts[0] || i == f_conn_verts[1] || i == f_conn_verts[2]);

        size_t eid = -1;

        // eid is the same as the lvid
        if (i == f_conn_verts[0]) eid = 2;
        if (i == f_conn_verts[1])
            eid = 0;
        else
            eid = 1;

        Tuple v_tuple = Tuple(i, eid, fid, *this);
        assert(v_tuple.is_valid(*this));
        all_vertices_tuples.push_back(v_tuple);
    }
    return all_vertices_tuples;
}

std::vector<TriMesh::Tuple> TriMesh::get_faces() const
{
    std::vector<Tuple> all_faces_tuples;
    all_faces_tuples.reserve(tri_capacity());
    for (size_t i = 0; i < tri_capacity(); i++) {
        if (m_tri_connectivity[i].m_is_removed) continue;
        // get the 3 vid
        const std::array<size_t, 3>& f_conn_verts = m_tri_connectivity[i].m_indices;
        size_t vid = f_conn_verts[0];
        Tuple f_tuple = Tuple(vid, 2, i, *this);
        assert(f_tuple.is_valid(*this));
        all_faces_tuples.push_back(f_tuple);
    }
    return all_faces_tuples;
}

std::vector<TriMesh::Tuple> TriMesh::get_edges() const
{
    std::vector<TriMesh::Tuple> all_edges_tuples;
    all_edges_tuples.reserve(tri_capacity() * 3);
    for (int fid = 0; fid < tri_capacity(); fid++) {
        if (m_tri_connectivity[fid].m_is_removed) {
            continue;
        }
        for (int j = 0; j < 3; j++) {
            const size_t loc_eid = (j + 2) % 3;
            const size_t vid = m_tri_connectivity[fid].m_indices[j];
            const Tuple tup(vid, loc_eid, fid, *this);
            if (tup.eid(*this) == 3 * fid + loc_eid) {
                all_edges_tuples.emplace_back(tup);
            }
        }
    }

    return all_edges_tuples;
}

TriMesh::Tuple TriMesh::tuple_from_edge(size_t vid1, size_t vid2, size_t fid) const
{
    auto a = m_tri_connectivity[fid].find(vid1);
    auto b = m_tri_connectivity[fid].find(vid2);
    assert(a != -1 && b != -1);
    // 0,1 - >2, 1,2-> 0, 0,2->1
    return Tuple(vid1, 3 - (a + b), fid, *this);
}

TriMesh::Tuple wmtk::TriMesh::tuple_from_vids(size_t vid0, size_t vid1, size_t vid2) const
{
    const auto& vf0 = m_vertex_connectivity[vid0];
    const auto& vf1 = m_vertex_connectivity[vid1];
    const auto& vf2 = m_vertex_connectivity[vid2];

    const std::vector<size_t> tris01 = set_intersection(vf0.m_conn_tris, vf1.m_conn_tris);
    const std::vector<size_t> tris012 = set_intersection(tris01, vf2.m_conn_tris);

    if (tris012.size() != 1) {
        log_and_throw_error("Cannot find face with vids ({},{},{})", vid0, vid1, vid2);
    }

    const size_t fid = tris012[0];

    const auto& tc = m_tri_connectivity[fid].m_indices;
    size_t local_vid = -1;
    for (int i = 0; i < 3; ++i) {
        if (tc[i] == vid0) {
            local_vid = i;
            break;
        }
    }
    assert(local_vid != -1);

    const size_t local_vid_next = (local_vid + 1) % 3;
    const size_t local_vid_prev = (local_vid + 2) % 3;
    if (tc[local_vid_next] == vid1) {
        return Tuple(vid0, local_vid_prev, fid, *this);
    } else {
        assert(tc[local_vid_prev] == vid1);
        return Tuple(vid0, local_vid_next, fid, *this);
    }

    Tuple t;


    return Tuple();
}

simplex::Vertex wmtk::TriMesh::simplex_from_vertex(const Tuple& t) const
{
    return simplex::Vertex(t.vid(*this));
}

simplex::Edge wmtk::TriMesh::simplex_from_edge(const Tuple& t) const
{
    return simplex::Edge(t.vid(*this), t.switch_vertex(*this).vid(*this));
}

simplex::Face wmtk::TriMesh::simplex_from_face(const Tuple& t) const
{
    return simplex_from_face(t.fid(*this));
}

simplex::Face wmtk::TriMesh::simplex_from_face(const size_t fid) const
{
    const auto vs = oriented_tri_vids(fid);
    return simplex::Face(vs[0], vs[1], vs[2]);
}

TriMesh::Tuple wmtk::TriMesh::tuple_from_simplex(const simplex::Face& s) const
{
    const auto& v = s.vertices();
    return tuple_from_vids(v[0], v[1], v[2]);
}

simplex::SimplexCollection wmtk::TriMesh::simplex_incident_triangles(const simplex::Vertex& v) const
{
    const auto fids = m_vertex_connectivity[v.vertices()[0]].m_conn_tris;
    simplex::SimplexCollection sc;

    for (const size_t fid : fids) {
        const auto vids = oriented_tri_vids(fid);
        sc.add(simplex::Face(vids[0], vids[1], vids[2]));
    }
    sc.sort_and_clean();
    return sc;
}

simplex::SimplexCollection wmtk::TriMesh::simplex_incident_triangles(const simplex::Edge& e) const
{
    const simplex::Vertex loc_v0(e.vertices()[0]);
    const simplex::Vertex v1(e.vertices()[1]);

    const auto sc0 = simplex_incident_triangles(loc_v0);
    const auto sc1 = simplex_incident_triangles(v1);

    return simplex::SimplexCollection::get_intersection(sc0, sc1);
}

simplex::SimplexCollection wmtk::TriMesh::simplex_link_vertices(const simplex::Vertex& v) const
{
    const auto tris = simplex_incident_triangles(v);
    simplex::SimplexCollection sc;
    sc.reserve_vertices(tris.faces().size() * 2);
    for (const simplex::Face& f : tris.faces()) {
        for (const size_t vid : f.vertices()) {
            if (vid != v.vertices()[0]) {
                sc.add(simplex::Vertex(vid));
            }
        }
    }
    sc.sort_and_clean();

    return sc;
}

simplex::SimplexCollection wmtk::TriMesh::simplex_link_vertices(const simplex::Edge& e) const
{
    const auto tris = simplex_incident_triangles(e);
    simplex::SimplexCollection sc;
    sc.reserve_vertices(tris.faces().size());
    for (const simplex::Face& f : tris.faces()) {
        sc.add(f.opposite_vertex(e));
    }
    sc.sort_and_clean();

    return sc;
}

simplex::SimplexCollection wmtk::TriMesh::simplex_link_edges(const simplex::Vertex& v) const
{
    const auto tris = simplex_incident_triangles(v);
    simplex::SimplexCollection sc;
    sc.reserve_edges(tris.faces().size());
    for (const simplex::Face& f : tris.faces()) {
        sc.add(f.opposite_edge(v));
    }
    sc.sort_and_clean();

    return sc;
}

// Atomically reserve `n` contiguous fresh triangle slots from the preallocated
// storage. Returns the first index, or -1 if that would exceed the preallocated
// capacity (the caller must abort the operation). See TetMesh::request_tet_slots.
long TriMesh::request_tri_slots(size_t n)
{
    if (n == 0) return (long)current_tri_size.load();
    const long cap = (long)m_tri_connectivity.size();
    // CAS loop: only advance the counter when there is room (see TetMesh version).
    long first = current_tri_size.load(std::memory_order_relaxed);
    do {
        if (first + (long)n > cap) return -1;
    } while (!current_tri_size.compare_exchange_weak(
        first,
        first + (long)n,
        std::memory_order_acq_rel,
        std::memory_order_relaxed));
    // Reset the handed-out slots to a clean state (the preallocated std::vector
    // keeps stale data in the spare region; the old tbb::collector regrew
    // fresh slots on consolidate).
    for (long i = first; i < first + (long)n; ++i) {
        m_tri_connectivity[i] = TriangleConnectivity{};
    }
    return first;
}

long TriMesh::request_vert_slots(size_t n)
{
    if (n == 0) return (long)current_vert_size.load();
    const long cap = (long)m_vertex_connectivity.size();
    long first = current_vert_size.load(std::memory_order_relaxed);
    do {
        if (first + (long)n > cap) return -1;
    } while (!current_vert_size.compare_exchange_weak(
        first,
        first + (long)n,
        std::memory_order_acq_rel,
        std::memory_order_relaxed));
    // Reset the handed-out vertex slots to a clean state (see request_tri_slots).
    for (long i = first; i < first + (long)n; ++i) {
        m_vertex_connectivity[i] = VertexConnectivity{};
    }
    return first;
}

size_t TriMesh::get_next_empty_slot_t()
{
    const long r = request_tri_slots(1);
    return r < 0 ? static_cast<size_t>(-1) : static_cast<size_t>(r);
}

size_t TriMesh::get_next_empty_slot_v()
{
    const long r = request_vert_slots(1);
    return r < 0 ? static_cast<size_t>(-1) : static_cast<size_t>(r);
}

bool TriMesh::swap_edge_before(const Tuple& t)
{
    // A swap replaces the edge by the one joining the two opposite vertices, which needs
    // exactly two of them: a boundary edge has none and a non-manifold edge has no
    // canonical pair. Rejecting is the only sensible answer for both, and it is what keeps
    // swap from having to reason about fans at all.
    if (!is_manifold_edge(t)) {
        return false;
    }
    const auto opps = t.switch_faces(*this);
    assert(opps.size() == 1);

    const size_t v3 = ((t.switch_edge(*this)).switch_vertex(*this)).vid(*this);
    const size_t v4 = opps[0].switch_edge(*this).switch_vertex(*this).vid(*this);
    if (!set_intersection(
             m_vertex_connectivity[v4].m_conn_tris,
             m_vertex_connectivity[v3].m_conn_tris)
             .empty())
        return false;
    return true;
}


// link check, prerequisite for edge collapse
bool wmtk::TriMesh::check_link_condition(const Tuple& edge) const
{
    assert(edge.is_valid(*this));
    size_t vid1 = edge.vid(*this);
    size_t vid2 = switch_vertex(edge).vid(*this);
    const auto vid1_ring = get_one_ring_edges_for_vertex(edge);
    const auto vid2_ring = get_one_ring_edges_for_vertex(switch_vertex(edge));


    size_t dummy = std::numeric_limits<size_t>::max();

    std::vector<size_t> lk_vid1; // link vertices of vid1
    std::vector<size_t> lk_vid2; // link vertices of vid2

    std::vector<std::pair<size_t, size_t>> lk_e_vid1; // link edges of vid1
    std::vector<std::pair<size_t, size_t>> lk_e_vid2; // link edges of vid2

    for (const Tuple& e_vid : vid1_ring) {
        // switch_faces(...).empty() asks whether the edge has exactly one face, which
        // is_boundary_edge now answers with a single array comparison instead of building
        // two vectors. This runs once per one-ring edge of both endpoints, on every
        // collapse attempt.
        if (is_boundary_edge(e_vid)) {
            lk_vid1.push_back(dummy);
            lk_e_vid1.emplace_back(e_vid.vid(*this), dummy);
        }
        lk_vid1.push_back(e_vid.vid(*this));
    }
    // collect link edges
    std::vector<Tuple> vid1_tris = get_one_ring_tris_for_vertex(edge);
    for (const Tuple& v1_tri_t : vid1_tris) {
        auto indices = m_tri_connectivity[v1_tri_t.fid(*this)].m_indices;
        auto l = m_tri_connectivity[v1_tri_t.fid(*this)].find(vid1);
        assert(l != -1);
        auto i0 = indices[(l + 1) % 3], i1 = indices[(l + 2) % 3];
        lk_e_vid1.emplace_back(std::min(i0, i1), std::max(i0, i1));
    }
    vector_unique(lk_vid1);

    for (const Tuple& e_vid : vid2_ring) {
        if (is_boundary_edge(e_vid)) {
            lk_vid2.push_back(dummy);
            lk_e_vid2.emplace_back(e_vid.vid(*this), dummy);
        }
        lk_vid2.push_back(e_vid.vid(*this));
    }
    // collect link edges
    std::vector<Tuple> vid2_tris = get_one_ring_tris_for_vertex(switch_vertex(edge));
    for (const Tuple& v2_tri_t : vid2_tris) {
        auto indices = m_tri_connectivity[v2_tri_t.fid(*this)].m_indices;
        auto l = m_tri_connectivity[v2_tri_t.fid(*this)].find(vid2);
        assert(l != -1);
        auto i0 = indices[(l + 1) % 3], i1 = indices[(l + 2) % 3];
        lk_e_vid2.emplace_back(std::min(i0, i1), std::max(i0, i1));
    }
    vector_unique(lk_vid2);


    auto lk_vid12 = set_intersection(lk_vid1, lk_vid2);
    std::vector<size_t> lk_edge;
    lk_edge.push_back((edge.switch_edge(*this)).switch_vertex(*this).vid(*this));
    if (is_boundary_edge(edge)) {
        lk_edge.push_back(dummy);
    } else {
        for (const Tuple& opp : edge.switch_faces(*this)) {
            lk_edge.push_back(opp.switch_edge(*this).switch_vertex(*this).vid(*this));
        }
    }
    vector_sort(lk_edge);
    bool v_link =
        (lk_vid12.size() == lk_edge.size() &&
         std::equal(lk_vid12.begin(), lk_vid12.end(), lk_edge.begin()));

    // check edge link condition
    // in 2d edge link for an edge is always empty

    if (!v_link) {
        return false;
    }

    // bool e_link = true;
    std::vector<std::pair<size_t, size_t>> res;
    std::sort(lk_e_vid1.begin(), lk_e_vid1.end());
    std::sort(lk_e_vid2.begin(), lk_e_vid2.end());
    std::set_intersection(
        lk_e_vid1.begin(),
        lk_e_vid1.end(),
        lk_e_vid2.begin(),
        lk_e_vid2.end(),
        std::back_inserter(res));
    if (res.size() > 0) {
        return false;
    }

    return true;
}

int TriMesh::release_vertex_mutex_in_stack()
{
    int num_released = 0;
    for (int i = (int)mutex_release_stack.local().size() - 1; i >= 0; i--) {
        unlock_vertex_mutex(mutex_release_stack.local()[i]);
        num_released++;
    }
    mutex_release_stack.local().clear();
    return num_released;
}

bool TriMesh::try_set_vertex_mutex_two_ring(const Tuple& v, int threadid)
{
    for (const Tuple& v_one_ring : get_one_ring_edges_for_vertex(v)) {
        if (m_vertex_mutex[v_one_ring.vid(*this)].get_owner() == threadid) {
            continue;
        }
        if (try_set_vertex_mutex(v_one_ring, threadid)) {
            mutex_release_stack.local().push_back(v_one_ring.vid(*this));
            for (const Tuple& v_two_ring : get_one_ring_edges_for_vertex(v_one_ring)) {
                if (m_vertex_mutex[v_two_ring.vid(*this)].get_owner() == threadid) {
                    continue;
                }
                if (try_set_vertex_mutex(v_two_ring, threadid)) {
                    mutex_release_stack.local().push_back(v_two_ring.vid(*this));
                } else {
                    return false;
                }
            }
        } else {
            return false;
        }
    }
    return true;
}

bool TriMesh::try_set_edge_mutex_two_ring(const Tuple& e, int threadid)
{
    Tuple v1 = e;
    bool release_flag = false;

    // try v1
    if (m_vertex_mutex[v1.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v1, threadid)) {
            mutex_release_stack.local().push_back(v1.vid(*this));
        } else {
            release_flag = true;
        }
    }

    if (!v1.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v2
    Tuple v2 = switch_vertex(e);
    if (m_vertex_mutex[v2.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v2, threadid)) {
            mutex_release_stack.local().push_back(v2.vid(*this));
        } else {
            release_flag = true;
        }
    }
    if (!v2.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v1 two ring
    release_flag = !try_set_vertex_mutex_two_ring(v1, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v2 two ring
    release_flag = !try_set_vertex_mutex_two_ring(v2, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    return true;
}

bool wmtk::TriMesh::try_set_vertex_mutex_one_ring(const Tuple& v, int threadid)
{
    auto& stack = mutex_release_stack.local();
    auto vid = v.vid(*this);
    if (m_vertex_mutex[vid].get_owner() != threadid) {
        if (try_set_vertex_mutex(v, threadid)) {
            stack.push_back(vid);
            for (auto v_one_ring : get_one_ring_vids_for_vertex_duplicate(vid)) {
                if (m_vertex_mutex[v_one_ring].get_owner() != threadid) {
                    if (try_set_vertex_mutex(v_one_ring, threadid)) {
                        stack.push_back(v_one_ring);
                    } else {
                        release_vertex_mutex_in_stack();
                        return false;
                    }
                }
            }
        } else {
            release_vertex_mutex_in_stack();
            return false;
        }
    }
    return true;
}

bool TriMesh::try_set_face_mutex_one_ring(const Tuple& f, int threadid)
{
    const auto verts = oriented_tri_vertices(f);
    for (const Tuple& v : verts) {
        if (m_vertex_mutex[v.vid(*this)].get_owner() != threadid) {
            release_vertex_mutex_in_stack();
            return false;
        }
        if (try_set_vertex_mutex(v, threadid)) {
            mutex_release_stack.local().push_back(v.vid(*this));
        } else {
            release_vertex_mutex_in_stack();
            return false;
        }
        if (!v.is_valid(*this)) {
            release_vertex_mutex_in_stack();
            return false;
        }
    }

    for (const Tuple& v : verts) {
        if (!try_set_vertex_mutex_one_ring(v, threadid)) {
            release_vertex_mutex_in_stack();
            return false;
        }
    }

    return true;
}

void wmtk::TriMesh::for_each_edge(const std::function<void(const TriMesh::Tuple&)>& func)
{
    threading::parallel_for(
        threading::range(0, tri_capacity()),
        [&](const threading::range& r) {
            for (size_t i = r.begin(); i < r.end(); i++) {
                if (!tuple_from_tri(i).is_valid(*this)) {
                    continue;
                }
                for (int j = 0; j < 3; j++) {
                    auto tup = tuple_from_edge(i, j);
                    if (tup.eid(*this) == 3 * i + j) {
                        func(tup);
                    }
                }
            }
        },
        NUM_THREADS);
}

void wmtk::TriMesh::for_each_vertex(const std::function<void(const TriMesh::Tuple&)>& func)
{
    threading::parallel_for(
        threading::range(0, vert_capacity()),
        [&](const threading::range& r) {
            for (size_t i = r.begin(); i < r.end(); i++) {
                auto tup = tuple_from_vertex(i);
                if (!tup.is_valid(*this)) {
                    continue;
                }
                func(tup);
            }
        },
        NUM_THREADS);
}

void wmtk::TriMesh::for_each_face(const std::function<void(const TriMesh::Tuple&)>& func)
{
    threading::parallel_for(
        threading::range(0, tri_capacity()),
        [&](const threading::range& r) {
            for (size_t i = r.begin(); i < r.end(); i++) {
                auto tup = tuple_from_tri(i);
                if (!tup.is_valid(*this)) {
                    continue;
                }
                func(tup);
            }
        },
        NUM_THREADS);
}


simplex::SimplexCollection TriMesh::get_surface_edges_for_vertex(const size_t vid) const
{
    using namespace simplex;

    SimplexCollection sc;

    if (!vertex_is_on_surface(vid)) {
        // no face can be on the surface if the vertex is not on the surface
        return sc;
    }

    const auto edges = get_one_ring_edges_for_vertex(vid);
    for (const Tuple& t : edges) {
        const simplex::Edge e(vid, t.vid(*this));
        if (get_order_of_edge(e.vertices()) == 1) {
            sc.add(e);
        }
    }

    sc.sort_and_clean();

    return sc;
}

size_t TriMesh::get_order_of_edge(const std::array<size_t, 2>& vids) const
{
    return edge_is_on_surface(vids) ? 1 : 0;
}

size_t TriMesh::get_order_of_vertex(const size_t vid) const
{
    const auto edges = get_one_ring_edges_for_vertex(vid);
    size_t surface_count = 0;
    for (const Tuple& t : edges) {
        if (get_order_of_edge({{vid, t.vid(*this)}}) > 0) {
            ++surface_count;
        }
    }
    if (surface_count == 0) {
        // vertex is not on the surface
        return 0;
    }
    if (surface_count == 2) {
        // vertex is on the surface
        return 1;
    }
    // vertex is on the surface boundary or non-manifold
    return 2;
}

bool TriMesh::substructure_link_condition(const Tuple& e_tuple) const
{
    const size_t u_id = e_tuple.vid(*this);
    const size_t v_id = e_tuple.switch_vertex(*this).vid(*this);

    using namespace simplex;

    const size_t edge_order = get_order_of_edge({{u_id, v_id}});
    const size_t u_order = get_order_of_vertex(u_id);
    const size_t v_order = get_order_of_vertex(v_id);

    // If the edge is lower order than both vertices, we know for sure that this edge must not
    // be collapsed. Example: edge in space (order 0) connecting two surfaces (order 1).
    // This check also covers the case that both vertices are order 3
    if (edge_order < u_order && edge_order < v_order) {
        return false;
    }

    const auto u_locs = get_one_ring_fids_for_vertex(u_id);
    const auto v_locs = get_one_ring_fids_for_vertex(v_id);
    const auto e_locs = set_intersection(u_locs, v_locs);

    SimplexCollection link_u_0;
    SimplexCollection link_u_1;
    SimplexCollection link_v_0;
    SimplexCollection link_v_1;
    SimplexCollection link_e_0;
    SimplexCollection link_e_1;

    constexpr size_t w_id = -1; // dummy vertex
    const Vertex w(w_id);

    const SimplexCollection u_surface_edges = get_surface_edges_for_vertex(u_id);
    const SimplexCollection v_surface_edges = get_surface_edges_for_vertex(v_id);

    // vertex u links
    {
        const Vertex u(u_id);
        link_u_0.reserve_edges(u_locs.size());
        link_u_0.reserve_vertices(u_locs.size() * 2);
        for (const size_t fid : u_locs) {
            const Face f = simplex_from_face(fid);
            const Edge e = f.opposite_edge(u);
            link_u_0.add_with_faces(e);
        }

        link_u_1.reserve_edges(u_surface_edges.faces().size());
        link_u_1.reserve_vertices(u_surface_edges.faces().size() * 2);

        for (const Edge& e : u_surface_edges.edges()) {
            const Face fw(e, w_id);
            const Edge ew = fw.opposite_edge(u);
            link_u_0.add_with_faces(ew);

            const Vertex e_opp = e.opposite_vertex(u);
            link_u_1.add(e_opp);
        }
        link_u_0.sort_and_clean();
        link_u_1.sort_and_clean();
    }
    // vertex v links
    {
        const Vertex v(v_id);
        link_v_0.reserve_edges(v_locs.size());
        link_v_0.reserve_vertices(v_locs.size() * 2);
        for (const size_t fid : v_locs) {
            const Face f = simplex_from_face(fid);
            const Edge e = f.opposite_edge(v);
            link_v_0.add_with_faces(e);
        }

        link_v_1.reserve_edges(v_surface_edges.faces().size());
        link_v_1.reserve_vertices(v_surface_edges.faces().size() * 2);

        for (const Edge& e : v_surface_edges.edges()) {
            const Face fw(e, w_id);
            const Edge ew = fw.opposite_edge(v);
            link_v_0.add_with_faces(ew);

            const Vertex e_opp = e.opposite_vertex(v);
            link_v_1.add(e_opp);
        }
        link_v_0.sort_and_clean();
        link_v_1.sort_and_clean();
    }
    // edge links
    {
        const Edge e(u_id, v_id);
        link_e_0.reserve_edges(e_locs.size());
        link_e_0.reserve_vertices(e_locs.size() * 2);
        for (const size_t fid : e_locs) {
            const Face face = simplex_from_face(fid);
            const Vertex v_opp = face.opposite_vertex(e);
            link_e_0.add(v_opp);
        }

        if (edge_order > 0) {
            link_e_0.add(w);
        }
        link_e_0.sort_and_clean();
    }

    const auto link_uv_0 = SimplexCollection::get_intersection(link_u_0, link_v_0);
    if (link_uv_0 != link_e_0) {
        return false;
    }
    const auto link_uv_1 = SimplexCollection::get_intersection(link_u_1, link_v_1);
    if (!link_uv_1.empty()) {
        return false;
    }

    return true;
}