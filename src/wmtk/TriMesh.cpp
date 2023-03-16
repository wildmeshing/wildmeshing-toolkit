#include <igl/is_edge_manifold.h>
#include <igl/writeDMAT.h>
#include <wmtk/TriMesh.h>
#include <wmtk/TriMeshOperation.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include "wmtk/utils/VectorUtils.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/parallel_for.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

using namespace wmtk;

void TriMesh::copy_connectivity(const TriMesh& o)
{
    // auto l = std::scoped_lock(vertex_connectivity_lock, tri_connectivity_lock,
    // o.vertex_connectivity_lock, o.tri_connectivity_lock);
    //  explicitly make sure that the connectivity data is copied and sized properly
    m_vertex_connectivity = o.m_vertex_connectivity;
    m_tri_connectivity = o.m_tri_connectivity;
    current_vert_size.store(o.current_vert_size.load());
    current_tri_size.store(o.current_tri_size.load());
    m_vertex_mutex.grow_to_at_least(o.m_vertex_connectivity.size());
}
TriMesh::TriMesh() {}
TriMesh::~TriMesh() {}

void TriMesh::Tuple::update_hash(const TriMesh& m)
{
    assert(m_fid < m.m_tri_connectivity.size());
    m_hash = m.m_tri_connectivity[m_fid].hash;
}

std::string TriMesh::Tuple::info() const
{
    return fmt::format("tuple: v{} e{} f{} (h{})", m_vid, m_eid, m_fid, m_hash);
}

void TriMesh::Tuple::print_info() const
{
    logger().trace("{}", info());
}

size_t TriMesh::Tuple::eid_unsafe(const TriMesh& m) const
{
    return m_fid * 3 + m_eid;
}

size_t TriMesh::Tuple::eid(const TriMesh& m) const
{
    if (switch_face(m).has_value()) {
        size_t fid2 = switch_face(m)->fid(m);
        size_t min_fid = std::min(m_fid, fid2);
        if (min_fid == fid2) {
            assert(fid2 < m.m_tri_connectivity.size());
            int i = m.m_tri_connectivity[fid2].find(m_vid);
            int j = m.m_tri_connectivity[fid2].find(switch_vertex(m).vid(m));
            return min_fid * 3 + 3 - i - j;
        }
    }
    return m_fid * 3 + m_eid;
}


TriMesh::Tuple TriMesh::Tuple::switch_vertex(const TriMesh& m) const
{
    assert(is_valid(m));

    const int v0 = m.m_tri_connectivity[m_fid][0];
    const int v1 = m.m_tri_connectivity[m_fid][1];
    const int v2 = m.m_tri_connectivity[m_fid][2];

    Tuple loc = *this;
    switch (m_eid) {
    case 0:
        assert(m_vid == v1 || m_vid == v2);
        loc.m_vid = m_vid == v1 ? v2 : v1;
        break;
    case 1:
        assert(m_vid == v0 || m_vid == v2);
        loc.m_vid = m_vid == v0 ? v2 : v0;
        break;
    case 2:
        assert(m_vid == v0 || m_vid == v1);
        loc.m_vid = m_vid == v0 ? v1 : v0;
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

    const size_t v0 = m_vid;
    const size_t v1 = this->switch_vertex(m).m_vid;

    // Intersect the 1-ring of the two vertices in the edge pointed by the tuple
    std::vector<size_t> v0_fids = m.m_vertex_connectivity[v0].m_conn_tris;
    std::vector<size_t> v1_fids = m.m_vertex_connectivity[v1].m_conn_tris;

    std::sort(v0_fids.begin(), v0_fids.end());
    std::sort(v1_fids.begin(), v1_fids.end());
    std::vector<int> fids;
    std::set_intersection(
        v0_fids.begin(),
        v0_fids.end(),
        v1_fids.begin(),
        v1_fids.end(),
        std::back_inserter(fids)); // make sure this is correct
    assert(fids.size() == 1 || fids.size() == 2);

    if (fids.size() == 1) return {};

    Tuple loc = *this;

    // There is a triangle on the other side
    if (fids.size() == 2) {
        // Find the fid of the triangle on the other side
        size_t fid2 = fids[0] == m_fid ? fids[1] : fids[0];
        loc.m_fid = fid2;

        // Get sorted local indices of the two vertices in the new triangle
        size_t lv0_2 = m.m_tri_connectivity[fid2].find(v0);
        assert(lv0_2 == 0 || lv0_2 == 1 || lv0_2 == 2);
        size_t lv1_2 = m.m_tri_connectivity[fid2].find(v1);
        assert(lv1_2 == 0 || lv1_2 == 1 || lv1_2 == 2);

        if (lv0_2 > lv1_2) std::swap(lv0_2, lv1_2);

        // Assign the edge id depending on the table
        if (lv0_2 == 0 && lv1_2 == 1) {
            loc.m_eid = 2;
        } else if (lv0_2 == 1 && lv1_2 == 2) {
            loc.m_eid = 0;
        } else if (lv0_2 == 0 && lv1_2 == 2) {
            loc.m_eid = 1;
        } else {
            assert(false);
        }

        loc.update_hash(m);
    }
    assert(loc.is_valid(m));
    return loc;
}

bool TriMesh::Tuple::is_ccw(const TriMesh& m) const
{
    if (m.m_tri_connectivity[m_fid][(m_eid + 1) % 3] == m_vid)
        return true;
    else
        return false;
}
bool TriMesh::Tuple::is_valid(const TriMesh& m) const
{
    if (m_fid >= m.m_tri_connectivity.size()) {
        return false;
    }
    if (m_vid >= m.m_vertex_connectivity.size()) {
        return false;
    }

    if (m_fid + 1 == 0) {
        return false;
    }

    if (m.m_vertex_connectivity[m_vid].m_is_removed) {
        return false;
    }

    if (m.m_tri_connectivity[m_fid].m_is_removed) {
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
    const int v0 = m.m_tri_connectivity[m_fid][0];
    const int v1 = m.m_tri_connectivity[m_fid][1];
    const int v2 = m.m_tri_connectivity[m_fid][2];
    switch (m_eid) {
    case 0: assert(m_vid == v1 || m_vid == v2); break;
    case 1: assert(m_vid == v0 || m_vid == v2); break;
    case 2: assert(m_vid == v0 || m_vid == v1); break;
    default: assert(false);
    }
#endif

    return true;
}

bool TriMesh::invariants(const std::vector<Tuple>&)
{
    return true;
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
    }
    return true;
}

bool wmtk::TriMesh::check_edge_manifold() const
{
    std::vector<size_t> count(tri_capacity() * 3, 0);
    auto faces = get_faces();
    for (Tuple& f : faces) {
        for (int i = 0; i < 3; i++) {
            size_t eid = f.eid(*this);
            assert(eid < count.size());
            count[eid]++;
            if (count[eid] > 2) {
                return false;
            }
            f = f.switch_vertex(*this).switch_edge(*this);
        }
    }
    for (size_t idx = 0; idx < count.size(); idx++) {
        if (count[idx] > 2) {
            return false;
        }
    }
    return true;
}

bool TriMesh::is_boundary_vertex(const TriMesh::Tuple& t) const
{
    auto ve = get_one_ring_edges_for_vertex(t);
    for (const auto& e : ve) {
        if (is_boundary_edge(e)) return true;
    }
    return false;
}


void TriMesh::consolidate_mesh()
{
    TriMeshConsolidateOperation op;
    op(*this, TriMesh::Tuple{});
}


std::vector<size_t> TriMesh::get_one_ring_vids_for_vertex_with_duplicates(const size_t& vid) const
{
    std::vector<size_t> one_ring;
    auto& conn_tri = m_vertex_connectivity[vid].m_conn_tris;

    one_ring.reserve(conn_tri.size() * 4);
    for (size_t tri : conn_tri) {
        for (auto j : m_tri_connectivity[tri].m_indices) {
            one_ring.push_back(j);
        }
    }

    return one_ring;
}

std::vector<size_t> TriMesh::get_one_ring_vids_for_vertex(const size_t& vid) const
{
    std::vector<size_t> one_ring;
    auto& conn_tri = m_vertex_connectivity[vid].m_conn_tris;

    one_ring.reserve(conn_tri.size() * 4);
    for (size_t tri : conn_tri) {
        for (auto j : m_tri_connectivity[tri].m_indices) {
            if (std::find(one_ring.begin(), one_ring.end(), j) == one_ring.end())
                one_ring.push_back(j);
        }
    }

    return one_ring;
}

std::vector<wmtk::TriMesh::Tuple> TriMesh::get_one_ring_tris_for_vertex(
    const wmtk::TriMesh::Tuple& t) const
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

std::vector<wmtk::TriMesh::Tuple> TriMesh::get_one_ring_edges_for_vertex(
    const wmtk::TriMesh::Tuple& t) const
{
    std::vector<Tuple> one_ring_edges;
    std::vector<size_t> one_ring_vertices;
    size_t vid = t.vid(*this);
    auto one_ring_tris = get_one_ring_tris_for_vertex(t);
    for (auto tri : one_ring_tris) {
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

    return one_ring_edges;
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
    std::array<size_t, 3> incident_verts;
    size_t fid = t.fid(*this);
    auto indices = m_tri_connectivity[fid].m_indices;

    incident_verts[0] = indices[0];
    incident_verts[1] = indices[1];
    incident_verts[2] = indices[2];

    return incident_verts;
}

void TriMesh::create_mesh(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris)
{
    // TODO: use more STL for filling in new vertex/tri
    // std::fill(m_vertex_connectivity.begin(), m_vertex_connectivity.end(), VertexConnectivity{});
    // std::fill(m_tri_connectivity.begin(), m_tri_connectivity.end(), TriangleConnectivity{});
    // m_vertex_connectivity.resize(n_vertices, {});
    // m_tri_connectivity.resize(tris.size(), {});
    m_tri_connectivity.m_attributes.resize(tris.size());

    size_t hash_cnt = 0;
    for (int i = 0; i < tris.size(); i++) {
        m_tri_connectivity[i].m_is_removed = false;

        m_tri_connectivity[i].m_indices = tris[i];

        m_tri_connectivity[i].hash = hash_cnt;
    }
    current_tri_size = tris.size();

    build_vertex_connectivity(n_vertices);

    // Resize user class attributes
    if (p_vertex_attrs) p_vertex_attrs->resize(vert_capacity());
    if (p_edge_attrs) p_edge_attrs->resize(tri_capacity() * 3);
    if (p_face_attrs) p_face_attrs->resize(tri_capacity());
}

void TriMesh::build_vertex_connectivity(size_t n_vertices)
{
    m_vertex_connectivity.m_attributes.resize(n_vertices);
    for (int i = 0; i < n_vertices; i++) {
        m_vertex_connectivity[i].m_is_removed = false;
    }
    for (int i = 0; i < m_tri_connectivity.size(); i++) {
        auto& tri_con = m_tri_connectivity[i];
        if (!tri_con.m_is_removed) {
            for (const size_t vind : tri_con.m_indices) {
                m_vertex_connectivity[vind].m_conn_tris.push_back(i);
            }
        }
    }
    current_vert_size = n_vertices;
    m_vertex_mutex.grow_to_at_least(n_vertices);
}

std::vector<TriMesh::Tuple> TriMesh::get_vertices() const
{
    const size_t n_vertices = vert_capacity();
    std::vector<Tuple> all_vertices_tuples;
    all_vertices_tuples.reserve(n_vertices);

    for (size_t i = 0; i < n_vertices; i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;

        const std::vector<size_t>& v_conn_fids = m_vertex_connectivity[i].m_conn_tris;

        assert(v_conn_fids.size() > 0);
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
    assert(tri_capacity() <= m_tri_connectivity.size());
    for (size_t i = 0; i < tri_capacity(); i++) {
        const TriangleConnectivity& tri_con = m_tri_connectivity[i];
        if (tri_con.m_is_removed) {
            continue;
        }
        // get the 3 vid
        const std::array<size_t, 3>& f_conn_verts = tri_con.m_indices;
        size_t vid = f_conn_verts[0];
        Tuple f_tuple = Tuple(vid, 2, i, *this);
        assert(f_tuple.is_valid(*this));
        all_faces_tuples.emplace_back(f_tuple);
    }
    return all_faces_tuples;
}

std::vector<TriMesh::Tuple> TriMesh::get_edges() const
{
    std::vector<TriMesh::Tuple> all_edges_tuples;
    all_edges_tuples.reserve(tri_capacity() * 3);
    assert(tri_capacity() <= m_tri_connectivity.size());
    for (int i = 0; i < tri_capacity(); i++) {
        const TriangleConnectivity& con = m_tri_connectivity[i];
        if (con.m_is_removed) continue;
        for (int j = 0; j < 3; j++) {
            size_t l = (j + 2) % 3;
            auto tup = Tuple(con.m_indices[j], l, i, *this);
            if (tup.eid(*this) == 3 * i + l) {
                all_edges_tuples.emplace_back(tup);
            }
        }
    }

    return all_edges_tuples;
}

TriMesh::Tuple TriMesh::init_from_edge(size_t vid1, size_t vid2, size_t fid) const
{
    auto a = m_tri_connectivity[fid].find(vid1);
    auto b = m_tri_connectivity[fid].find(vid2);
    assert(a != -1 && b != -1);
    // 0,1 - >2, 1,2-> 0, 0,2->1
    return Tuple(vid1, 3 - (a + b), fid, *this);
}


size_t TriMesh::get_next_empty_slot_t()
{
    while (current_tri_size + MAX_THREADS >= m_tri_connectivity.size() ||
           tri_connectivity_synchronizing_flag) {
        if (tri_connectivity_lock.try_lock()) {
            if (current_tri_size + MAX_THREADS < m_tri_connectivity.size()) {
                tri_connectivity_lock.unlock();
                break;
            }
            tri_connectivity_synchronizing_flag = true;
            auto current_capacity = m_tri_connectivity.size();
            if (p_edge_attrs) p_edge_attrs->resize(2 * current_capacity * 3);
            if (p_face_attrs) p_face_attrs->resize(2 * current_capacity);
            m_tri_connectivity.grow_to_at_least(2 * current_capacity);
            tri_connectivity_synchronizing_flag = false;
            tri_connectivity_lock.unlock();
            break;
        }
    }

    return current_tri_size++;
}

size_t TriMesh::get_next_empty_slot_v()
{
    while (current_vert_size + MAX_THREADS >= m_vertex_connectivity.size() ||
           vertex_connectivity_synchronizing_flag) {
        if (vertex_connectivity_lock.try_lock()) {
            if (current_vert_size + MAX_THREADS < m_vertex_connectivity.size()) {
                vertex_connectivity_lock.unlock();
                break;
            }
            vertex_connectivity_synchronizing_flag = true;
            auto current_capacity = m_vertex_connectivity.size();
            if (p_vertex_attrs) p_vertex_attrs->resize(2 * current_capacity);
            resize_mutex(2 * current_capacity);
            m_vertex_connectivity.grow_to_at_least(2 * current_capacity);
            vertex_connectivity_synchronizing_flag = false;
            vertex_connectivity_lock.unlock();
            break;
        }
    }

    return current_vert_size++;
}


int TriMesh::release_vertex_mutex_in_stack()
{
    int num_released = 0;
    for (int i = mutex_release_stack.local().size() - 1; i >= 0; i--) {
        unlock_vertex_mutex(mutex_release_stack.local()[i]);
        num_released++;
    }
    mutex_release_stack.local().clear();
    return num_released;
}

bool TriMesh::try_set_vertex_mutex_two_ring(const Tuple& v, int threadid)
{
    for (auto v_one_ring : get_one_ring_edges_for_vertex(v)) {
        if (m_vertex_mutex[v_one_ring.vid(*this)].get_owner() == threadid) continue;
        if (try_set_vertex_mutex(v_one_ring, threadid)) {
            mutex_release_stack.local().push_back(v_one_ring.vid(*this));
            for (auto v_two_ring : get_one_ring_edges_for_vertex(v_one_ring)) {
                if (m_vertex_mutex[v_two_ring.vid(*this)].get_owner() == threadid) continue;
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
            for (auto v_one_ring : get_one_ring_vids_for_vertex_with_duplicates(vid)) {
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

void wmtk::TriMesh::for_each_edge(const std::function<void(const TriMesh::Tuple&)>& func)
{
    tbb::task_arena arena(NUM_THREADS);
    arena.execute([&] {
        tbb::parallel_for(
            tbb::blocked_range<int>(0, tri_capacity()),
            [&](const tbb::blocked_range<int>& r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    if (!tuple_from_tri(i).is_valid(*this)) continue;
                    for (int j = 0; j < 3; j++) {
                        auto tup = tuple_from_edge(i, j);
                        if (tup.eid(*this) == 3 * i + j) {
                            func(tup);
                        }
                    }
                }
            });
    });
}

void wmtk::TriMesh::for_each_vertex(const std::function<void(const TriMesh::Tuple&)>& func)
{
    tbb::task_arena arena(NUM_THREADS);
    arena.execute([&] {
        tbb::parallel_for(
            tbb::blocked_range<int>(0, vert_capacity()),
            [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    auto tup = tuple_from_vertex(i);
                    if (!tup.is_valid(*this)) continue;
                    func(tup);
                }
            });
    });
}

void wmtk::TriMesh::for_each_face(const std::function<void(const TriMesh::Tuple&)>& func)
{
    tbb::task_arena arena(NUM_THREADS);
    arena.execute([&] {
        tbb::parallel_for(
            tbb::blocked_range<int>(0, tri_capacity()),
            [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    auto tup = tuple_from_tri(i);
                    if (!tup.is_valid(*this)) continue;
                    func(tup);
                }
            });
    });
}

void TriMesh::release_protected_connectivity()
{
    m_vertex_connectivity.end_protect();
    m_tri_connectivity.end_protect();
}

void TriMesh::release_protected_attributes()
{
    if (p_vertex_attrs) {
        // if (auto& wp_op_rec = p_operation_recorder.local(); !wp_op_rec.expired()) {
        //     p_vertex_attrs->record_updates(*wp_op_rec.lock(), "vertex");
        // }
        p_vertex_attrs->end_protect();
    }
    if (p_edge_attrs) {
        // if (auto& wp_op_rec = p_operation_recorder.local(); !wp_op_rec.expired()) {
        //     p_vertex_attrs->record_updates(*wp_op_rec.lock(), "edge");
        // }
        p_edge_attrs->end_protect();
    }
    if (p_face_attrs) {
        // if (auto& wp_op_rec = p_operation_recorder.local(); !wp_op_rec.expired()) {
        //     p_vertex_attrs->record_updates(*wp_op_rec.lock(), "face");
        // }
        p_face_attrs->end_protect();
    }
}
