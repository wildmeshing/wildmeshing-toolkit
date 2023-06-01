#include <igl/is_edge_manifold.h>
#include <igl/writeDMAT.h>
#include <wmtk/TriMesh.h>
#include <wmtk/TriMeshOperation.h>
#include <wmtk/operations/TriMeshConsolidateOperation.h>
#include <wmtk/operations/TriMeshEdgeCollapseOperation.h>
#include <wmtk/operations/TriMeshEdgeSplitOperation.h>
#include <wmtk/operations/TriMeshEdgeSwapOperation.h>
#include <wmtk/operations/TriMeshVertexSmoothOperation.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleUtils.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/parallel_for.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

using namespace wmtk;

std::map<std::string, std::shared_ptr<TriMeshOperation>> TriMesh::get_operations() const
{
    std::map<std::string, std::shared_ptr<TriMeshOperation>> r;
    auto add_operation = [&](auto&& op) { r[op->name()] = op; };
    add_operation(std::make_shared<wmtk::TriMeshEdgeCollapseOperation>());
    add_operation(std::make_shared<wmtk::TriMeshEdgeSwapOperation>());
    add_operation(std::make_shared<wmtk::TriMeshEdgeSplitOperation>());
    add_operation(std::make_shared<wmtk::TriMeshVertexSmoothOperation>());
    add_operation(std::make_shared<wmtk::TriMeshConsolidateOperation>());
    return r;
}

class TriMesh::VertexMutex
{
    tbb::spin_mutex mutex;
    int owner = std::numeric_limits<int>::max();

public:
    bool trylock() { return mutex.try_lock(); }

    void unlock()
    {
        reset_owner();
        mutex.unlock();
    }

    int get_owner() { return owner; }

    void set_owner(int n) { owner = n; }

    void reset_owner() { owner = std::numeric_limits<int>::max(); }
};
size_t TriMesh::get_valence_for_vertex(const Tuple& t) const
{
    return m_vertex_connectivity[t.vid(*this)].m_conn_tris.size();
}
bool TriMesh::try_set_vertex_mutex(const Tuple& v, int threadid)
{
    bool got = m_vertex_mutex[v.vid(*this)].trylock();
    if (got) m_vertex_mutex[v.vid(*this)].set_owner(threadid);
    return got;
}
bool TriMesh::try_set_vertex_mutex(size_t vid, int threadid)
{
    bool got = m_vertex_mutex[vid].trylock();
    if (got) m_vertex_mutex[vid].set_owner(threadid);
    return got;
}

void TriMesh::unlock_vertex_mutex(const Tuple& v)
{
    unlock_vertex_mutex(v.vid(*this));
}
void TriMesh::unlock_vertex_mutex(size_t vid)
{
    m_vertex_mutex[vid].unlock();
}

void TriMesh::resize_mutex(size_t v)
{
    m_vertex_mutex.grow_to_at_least(v);
}

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


bool TriMesh::invariants(const TriMeshOperation&)
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

// edges pointing from the one-ring vertex to the center vertex
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
    // m_vertex_connectivity.grow_to_at_least(n_vertices, {});
    // m_tri_connectivity.grow_to_at_least(tris.size(), {});
    m_tri_connectivity.m_attributes.grow_to_at_least(tris.size());

    size_t hash_cnt = 0;
    for (int i = 0; i < tris.size(); i++) {
        m_tri_connectivity[i].m_is_removed = false;

        m_tri_connectivity[i].m_indices = tris[i];

        m_tri_connectivity[i].hash = hash_cnt;
    }
    current_tri_size = tris.size();

    build_vertex_connectivity(n_vertices);

    // Resize user class attributes
    if (p_vertex_attrs) p_vertex_attrs->grow_to_at_least(vert_capacity());
    if (p_edge_attrs) p_edge_attrs->grow_to_at_least(tri_capacity() * 3);
    if (p_face_attrs) p_face_attrs->grow_to_at_least(tri_capacity());
}

void TriMesh::build_vertex_connectivity(size_t n_vertices)
{
    m_vertex_connectivity.m_attributes.grow_to_at_least(n_vertices);
    for (int i = 0; i < n_vertices; i++) {
        m_vertex_connectivity[i].m_is_removed = false;
    }
    for (int i = 0; i < m_tri_connectivity.size(); i++) {
        auto& tri_con = m_tri_connectivity[i];
        if (!tri_con.m_is_removed) {
            for (const size_t vind : tri_con.m_indices) {
                m_vertex_connectivity[vind].m_conn_tris.push_back(i);
                m_vertex_connectivity[vind].m_is_removed = false;
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
    const auto opt = init_from_edge_opt(vid1, vid2, fid);
    assert(opt.has_value());
    return opt.value();
}
std::optional<TriMesh::Tuple> TriMesh::init_from_edge_opt(size_t vid1, size_t vid2, size_t fid)
    const
{
    auto a = m_tri_connectivity[fid].find(vid1);
    auto b = m_tri_connectivity[fid].find(vid2);
    if (a == -1 || b == -1) {
        return {};
    }
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
            if (p_edge_attrs) p_edge_attrs->grow_to_at_least(2 * current_capacity * 3);
            if (p_face_attrs) p_face_attrs->grow_to_at_least(2 * current_capacity);
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
            if (p_vertex_attrs) p_vertex_attrs->grow_to_at_least(2 * current_capacity);
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

std::optional<size_t> TriMesh::release_protected_connectivity()
{
    m_vertex_connectivity.end_protect();
    return m_tri_connectivity.end_protect();
}

std::array<std::optional<size_t>, 3> TriMesh::release_protected_attributes()
{
    std::array<std::optional<size_t>, 3> updates;
    if (p_vertex_attrs) {
        updates[0] = p_vertex_attrs->end_protect();
    }
    if (p_edge_attrs) {
        updates[1] = p_edge_attrs->end_protect();
    }
    if (p_face_attrs) {
        updates[2] = p_face_attrs->end_protect();
    }
    return updates;
}

auto TriMesh::triangle_boundary_edge_tuples(const Tuple& triangle) const -> std::array<Tuple, 3>
{
    assert(triangle.is_valid(*this));
    const size_t fid = triangle.fid(*this);

    return {{tuple_from_edge(fid, 0), tuple_from_edge(fid, 1), tuple_from_edge(fid, 2)}};
}
void TriMesh::rollback_protected_attributes()
{
    if (p_vertex_attrs) p_vertex_attrs->rollback();
    if (p_edge_attrs) p_edge_attrs->rollback();
    if (p_face_attrs) p_face_attrs->rollback();
}

auto TriMesh::tuple_from_tri(size_t fid) const -> Tuple
{
    if (fid >= m_tri_connectivity.size() || m_tri_connectivity[fid].m_is_removed) return Tuple();
    auto vid = m_tri_connectivity[fid][0];
    return Tuple(vid, 1, fid, *this);
}

auto TriMesh::tuple_from_vertex(size_t vid) const -> Tuple
{
    auto fid = m_vertex_connectivity[vid][0];
    auto eid = m_tri_connectivity[fid].find(vid);
    return Tuple(vid, (eid + 1) % 3, fid, *this);
}

auto TriMesh::tuple_from_edge_vids_opt(size_t vid1, size_t vid2) const -> std::optional<Tuple>
{
    const auto fids = tri_fids_bounded_by_edge_vids(vid1, vid2);
    if (fids.size() > 0) {
        return init_from_edge_opt(vid1, vid2, fids[0]);
    }
    return {};
}

auto TriMesh::tuple_from_edge(size_t fid, size_t local_eid) const -> Tuple
{
    auto vid = m_tri_connectivity[fid][(local_eid + 1) % 3];
    return Tuple(vid, local_eid, fid, *this);
}

/**
 * @brief rollback the connectivity that are modified if any condition failed
 */
void TriMesh::rollback_protected_connectivity()
{
    m_vertex_connectivity.rollback();
    m_tri_connectivity.rollback();
}

void TriMesh::rollback_protected()
{
    rollback_protected_connectivity();
    rollback_protected_attributes();
}

void TriMesh::start_protected_attributes()
{
    if (p_vertex_attrs) p_vertex_attrs->begin_protect();
    if (p_edge_attrs) p_edge_attrs->begin_protect();
    if (p_face_attrs) p_face_attrs->begin_protect();
}
/**
 * @brief Start caching the connectivity that will be modified
 */
void TriMesh::start_protected_connectivity()
{
    m_vertex_connectivity.begin_protect();
    m_tri_connectivity.begin_protect();
}

auto TriMesh::tris_bounded_by_edge(const Tuple& edge) const -> std::vector<Tuple>
{
    std::vector<Tuple> ret;
    const std::vector<size_t> fids = tri_fids_bounded_by_edge(edge);
    const size_t v0 = edge.vid(*this);
    const size_t v1 = edge.switch_vertex(*this).vid(*this);
    ret.reserve(fids.size());
    std::transform(
        fids.begin(),
        fids.end(),
        std::back_inserter(ret),
        [&](const size_t fid) -> Tuple { return init_from_edge(v0, v1, fid); });
    return ret;
}

std::vector<size_t> TriMesh::tri_fids_bounded_by_edge(const Tuple& edge) const
{
    size_t v0 = edge.vid(*this);
    size_t v1 = edge.switch_vertex(*this).vid(*this);
    return tri_fids_bounded_by_edge_vids(v0, v1);
}

std::vector<size_t> TriMesh::tri_fids_bounded_by_edge_vids(size_t v0, size_t v1) const
{
    // get the fids
    const auto& f0 = m_vertex_connectivity[v0].m_conn_tris;

    const auto& f1 = m_vertex_connectivity[v1].m_conn_tris;

    // get the fids that will be modified
    return set_intersection(f0, f1);
}

auto TriMesh::start_protected_attributes_raii() -> ProtectedAttributeRAII
{
    auto get_opt = [](AbstractAttributeCollection* ptr) -> AttributeCollectionProtectRAII {
        if (ptr != nullptr) {
            return AttributeCollectionProtectRAII(*ptr);
        } else {
            return AttributeCollectionProtectRAII();
        }
    };
    return std::array<AttributeCollectionProtectRAII, 3>{
        {get_opt(p_vertex_attrs), get_opt(p_edge_attrs), get_opt(p_face_attrs)}};
}
auto TriMesh::start_protected_connectivity_raii() -> ProtectedConnectivityRAII
{
    return std::array<AttributeCollectionProtectRAII, 2>{
        {AttributeCollectionProtectRAII(m_vertex_connectivity),
         AttributeCollectionProtectRAII(m_tri_connectivity)}};
}

