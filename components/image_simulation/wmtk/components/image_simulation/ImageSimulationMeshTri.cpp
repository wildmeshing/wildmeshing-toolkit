#include "ImageSimulationMeshTri.hpp"

#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
#include <igl/predicates/predicates.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <paraviewo/VTUWriter.hpp>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/TupleUtils.hpp>


namespace {
static int debug_print_counter = 0;
}

namespace wmtk::components::image_simulation::tri {

auto renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, TriMesh::Tuple>>();
    for (auto& e : edges) optup.emplace_back(op, e);
    return optup;
};


auto edge_locker = [](auto& m, const auto& e, int task_id) {
    // TODO: this should not be here
    return m.try_set_edge_mutex_two_ring(e, task_id);
};

// TODO: this should not be here
void ImageSimulationMeshTri::partition_mesh()
{
    auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
    for (auto i = 0; i < m_vertex_partition_id.size(); i++)
        m_vertex_attribute[i].partition_id = m_vertex_partition_id[i];
}

// TODO: morton should not be here, but inside wmtk
void ImageSimulationMeshTri::partition_mesh_morton()
{
    if (NUM_THREADS == 0) return;
    wmtk::logger().info("Number of parts: {} by morton", NUM_THREADS);

    tbb::task_arena arena(NUM_THREADS);

    arena.execute([&] {
        std::vector<Vector2d> V_v(vert_capacity());

        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, V_v.size()),
            [&](tbb::blocked_range<size_t> r) {
                for (size_t i = r.begin(); i < r.end(); i++) {
                    V_v[i] = m_vertex_attribute[i].m_pos;
                }
            });

        struct sortstruct
        {
            size_t order;
            Resorting::MortonCode64 morton;
        };

        std::vector<sortstruct> list_v;
        list_v.resize(V_v.size());
        const int multi = 1000;
        // since the morton code requires a correct scale of input vertices,
        //  we need to scale the vertices if their coordinates are out of range
        std::vector<Vector2d> V = V_v; // this is for rescaling vertices
        Vector2d vmin, vmax;
        vmin = V.front();
        vmax = V.front();

        for (size_t j = 0; j < V.size(); j++) {
            for (int i = 0; i < 3; i++) {
                vmin(i) = std::min(vmin(i), V[j](i));
                vmax(i) = std::max(vmax(i), V[j](i));
            }
        }

        Vector2d center = (vmin + vmax) / 2;

        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, V.size()),
            [&](tbb::blocked_range<size_t> r) {
                for (size_t i = r.begin(); i < r.end(); i++) {
                    V[i] = V[i] - center;
                }
            });

        Vector2d scale_point =
            vmax - center; // after placing box at origin, vmax and vmin are symetric.

        double xscale, yscale;
        xscale = fabs(scale_point[0]);
        yscale = fabs(scale_point[1]);
        double scale = std::max(xscale, yscale);
        if (scale > 300) {
            tbb::parallel_for(
                tbb::blocked_range<size_t>(0, V.size()),
                [&](tbb::blocked_range<size_t> r) {
                    for (size_t i = r.begin(); i < r.end(); i++) {
                        V[i] = V[i] / scale;
                    }
                });
        }

        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, V.size()),
            [&](tbb::blocked_range<size_t> r) {
                for (size_t i = r.begin(); i < r.end(); i++) {
                    list_v[i].morton = Resorting::MortonCode64(
                        int(V[i][0] * multi),
                        int(V[i][1] * multi),
                        int(V[i][2] * multi));
                    list_v[i].order = i;
                }
            });

        const auto morton_compare = [](const sortstruct& a, const sortstruct& b) {
            return (a.morton < b.morton);
        };

        tbb::parallel_sort(list_v.begin(), list_v.end(), morton_compare);

        size_t interval = list_v.size() / NUM_THREADS + 1;

        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, list_v.size()),
            [&](tbb::blocked_range<size_t> r) {
                for (size_t i = r.begin(); i < r.end(); i++) {
                    m_vertex_attribute[list_v[i].order].partition_id = i / interval;
                }
            });
    });
}

double ImageSimulationMeshTri::get_length2(const Tuple& l) const
{
    const auto vs = get_edge_vids(l);
    const Vector2d& p0 = m_vertex_attribute.at(vs[0]).m_pos;
    const Vector2d& p1 = m_vertex_attribute.at(vs[1]).m_pos;

    return (p1 - p0).squaredNorm();
}

void ImageSimulationMeshTri::init_from_image(
    const MatrixXd& V,
    const MatrixXi& T,
    const MatrixXi& T_tags)

{
    assert(V.cols() == 2);
    assert(T.cols() == 3);
    assert(T_tags.rows() == T.rows());

    init(T);

    assert(check_mesh_connectivity_validity());

    m_tags_count = T_tags.cols();

    m_vertex_attribute.m_attributes.resize(V.rows());
    m_face_attribute.m_attributes.resize(T.rows());
    m_edge_attribute.m_attributes.resize(T.rows() * 3);

    for (int i = 0; i < vert_capacity(); i++) {
        m_vertex_attribute[i].m_pos = V.row(i);
    }

    // init quality and check for inverted mesh
    bool is_mesh_inverted = false;
    for (const Tuple& t : get_faces()) {
        if (is_mesh_inverted ^ is_inverted(t)) {
            if (!is_mesh_inverted) {
                is_mesh_inverted = true;
            } else {
                log_and_throw_error("Tets with different orientations in the input!");
            }
        }
        m_face_attribute[t.fid(*this)].m_quality = get_quality(t);
    }

    if (is_mesh_inverted) {
        log_and_throw_error(
            "Input mesh is fully inverted! This should not happen... Might be a bug.");
    }


    // add tags
    for (size_t i = 0; i < T_tags.rows(); ++i) {
        m_face_attribute[i].tags.resize(m_tags_count);
        for (size_t j = 0; j < m_tags_count; ++j) {
            m_face_attribute[i].tags[j] = T_tags(i, j);
        }
    }

    init_surfaces_and_boundaries();
}

void ImageSimulationMeshTri::init_surfaces_and_boundaries()

{
    const auto edges = get_edges();
    logger().info("#E = {}", edges.size());

    // tag surface edges and vertices
    std::vector<Vector2i> tempE;
    for (const Tuple& e : edges) {
        SmartTuple ee(*this, e);

        const auto t_opp = ee.switch_face();
        if (!t_opp) {
            continue;
        }

        bool has_two_tags = false;

        for (size_t j = 0; j < m_tags_count; ++j) {
            const int64_t tag0 = m_face_attribute[ee.fid()].tags[j];
            const int64_t tag1 = m_face_attribute[t_opp.value().fid()].tags[j];

            if (tag0 != tag1) {
                has_two_tags = true;
                break;
            }
        }

        if (!has_two_tags) {
            continue;
        }

        m_edge_attribute[ee.eid()].m_is_surface_fs = 1;

        const size_t v1 = ee.vid();
        const size_t v2 = ee.switch_vertex().vid();
        m_vertex_attribute[v1].m_is_on_surface = true;
        m_vertex_attribute[v2].m_is_on_surface = true;

        tempE.emplace_back(v1, v2);
    }

    if (!m_envelope) {
        logger().info("Init envelope from face tags");
        // build envelopes
        std::vector<Vector2d> tempV(vert_capacity());
        for (int i = 0; i < vert_capacity(); i++) {
            tempV[i] = m_vertex_attribute[i].m_pos;
        }

        m_V_envelope = tempV;
        m_E_envelope = tempE;
        m_envelope = std::make_shared<SampleEnvelope>();
        m_envelope->init(m_V_envelope, m_E_envelope, m_envelope_eps);
    }

    // All surface edges must be inside the envelope
    {
        logger().info("Envelope sanity check");
        const auto surf_edges = get_edges_by_condition([](auto& f) { return f.m_is_surface_fs; });
        for (const auto& verts : surf_edges) {
            std::array<Vector2d, 2> pp = {
                m_vertex_attribute[verts[0]].m_pos,
                m_vertex_attribute[verts[1]].m_pos};
            if (m_envelope->is_outside(pp)) {
                log_and_throw_error("Edge {} is outside!", verts);
            }
        }
        logger().info("Envelope sanity check done");
    }

    // track bounding box
    for (size_t i = 0; i < edges.size(); i++) {
        const auto vids = get_edge_vids(edges[i]);
        int on_bbox = -1;
        for (int k = 0; k < 2; k++) {
            if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_min[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_min[k]) {
                on_bbox = k * 2;
                break;
            }
            if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_max[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_max[k]) {
                on_bbox = k * 2 + 1;
                break;
            }
        }
        if (on_bbox < 0) {
            continue;
        }
        assert(!edges[i].switch_face(*this)); // face must be on boundary

        const size_t eid = edges[i].eid(*this);
        m_edge_attribute[eid].m_is_bbox_fs = on_bbox;

        for (const size_t vid : vids) {
            m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
        }
    }

    for_each_vertex(
        [&](auto& v) { wmtk::vector_unique(m_vertex_attribute[v.vid(*this)].on_bbox_faces); });
}

bool ImageSimulationMeshTri::adjust_sizing_field_serial(double max_energy)
{
    log_and_throw_error("not implemented");
}

void ImageSimulationMeshTri::write_msh(std::string file)
{
    log_and_throw_error("not implemented");
}

void ImageSimulationMeshTri::write_vtu(const std::string& path)
{
    // consolidate_mesh();
    const std::string out_path = path + ".vtu";
    logger().info("Write {}", out_path);
    const auto& vs = get_vertices();
    const auto& faces = get_faces();
    const auto edges = get_edges_by_condition([](auto& f) { return f.m_is_surface_fs; });

    Eigen::MatrixXd V(vert_capacity(), 2);
    Eigen::MatrixXi F(tri_capacity(), 3);
    Eigen::MatrixXi E(edges.size(), 2);

    V.setZero();
    F.setZero();
    E.setZero();

    Eigen::VectorXd v_sizing_field(vert_capacity());
    v_sizing_field.setZero();

    std::vector<MatrixXd> tags(m_tags_count, MatrixXd(tri_capacity(), 1));
    Eigen::MatrixXd amips(tri_capacity(), 1);

    int index = 0;
    for (const Tuple& t : faces) {
        size_t tid = t.fid(*this);
        for (size_t j = 0; j < m_tags_count; ++j) {
            tags[j](index, 0) = m_face_attribute[tid].tags[j];
        }
        amips(index, 0) = m_face_attribute[tid].m_quality;

        const auto& vs = oriented_tri_vids(t);
        for (int j = 0; j < 3; j++) {
            F(index, j) = vs[j];
        }
        ++index;
    }

    for (size_t i = 0; i < edges.size(); ++i) {
        for (size_t j = 0; j < 2; ++j) {
            E(i, j) = edges[i][j];
        }
    }

    for (const Tuple& v : vs) {
        const size_t vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_pos;
        v_sizing_field[vid] = m_vertex_attribute[vid].m_sizing_scalar;
    }

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();

    for (size_t j = 0; j < m_tags_count; ++j) {
        writer->add_cell_field(fmt::format("tag_{}", j), tags[j]);
    }
    writer->add_cell_field("quality", amips);
    writer->add_field("sizing_field", v_sizing_field);
    writer->write_mesh(path + ".vtu", V, F);

    // surface
    {
        const auto surf_out_path = path + "_surf.vtu";
        std::shared_ptr<paraviewo::ParaviewWriter> surf_writer;
        surf_writer = std::make_shared<paraviewo::VTUWriter>();
        surf_writer->add_field("sizing_field", v_sizing_field);

        logger().info("Write {}", surf_out_path);
        surf_writer->write_mesh(surf_out_path, V, E);
    }
}

std::vector<std::array<size_t, 2>> ImageSimulationMeshTri::get_edges_by_condition(
    std::function<bool(const EdgeAttributes&)> cond)
{
    std::vector<std::array<size_t, 2>> res;
    for (const Tuple& e : get_edges()) {
        size_t eid = e.eid(*this);
        if (cond(m_edge_attribute[eid])) {
            res.push_back({{e.vid(*this), e.switch_vertex(*this).vid(*this)}});
        }
    }
    return res;
}

void ImageSimulationMeshTri::split_all_edges()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (const Tuple& loc : get_edges()) {
        collect_all_ops.emplace_back("edge_split", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("edge split prepare time: {:.4}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples =
            [](const ImageSimulationMeshTri& m, std::string op, const auto& newts) {
                std::vector<std::pair<std::string, TriMesh::Tuple>> op_tups;
                for (const auto& t : newts) {
                    op_tups.emplace_back(op, t);
                    op_tups.emplace_back(op, t.switch_edge(m));
                    op_tups.emplace_back(op, t.switch_vertex(m).switch_edge(m));
                }
                return op_tups;
            };

        executor.priority = [&](const ImageSimulationMeshTri& m, std::string op, const Tuple& t) {
            return m.get_length2(t);
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [&](const ImageSimulationMeshTri& m, const auto& ele) {
            auto [weight, op, tup] = ele;
            auto length = m.get_length2(tup);
            if (length != weight) {
                return false;
            }
            //
            size_t v1_id = tup.vid(*this);
            size_t v2_id = tup.switch_vertex(*this).vid(*this);
            const auto& VA = m_vertex_attribute;
            double sizing_ratio = 0.5 * (VA[v1_id].m_sizing_scalar + VA[v2_id].m_sizing_scalar);
            if (length < m_params.splitting_l2 * sizing_ratio * sizing_ratio) {
                return false;
            }
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor =
            wmtk::ExecutePass<ImageSimulationMeshTri, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [&](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge split operation time parallel: {:.4}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<ImageSimulationMeshTri, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge split operation time serial: {:.4}s", time);
    }
}

bool ImageSimulationMeshTri::split_edge_before(const Tuple& loc0)
{
    auto& cache = split_cache.local();

    cache.changed_edges.clear();
    cache.faces.clear();

    cache.v1_id = loc0.vid(*this);
    cache.v2_id = loc0.switch_vertex(*this).vid(*this);

    cache.old_e_attrs = m_edge_attribute[loc0.eid(*this)];

    const simplex::Edge edge(cache.v1_id, cache.v2_id);

    auto faces = get_incident_fids_for_edge(loc0);
    for (const size_t fid : faces) {
        auto vs = oriented_tri_vids(fid);
        for (int j = 0; j < 3; j++) {
            const simplex::Edge e(vs[j], vs[(j + 1) % 3]);
            if (e == edge) {
                continue;
            }
            if (cache.changed_edges.count(e) != 0) {
                continue;
            }
            auto [_, eid] = tuple_from_edge(e.vertices());
            cache.changed_edges[e] = m_edge_attribute[eid];
        }
    }

    // store tet attributes
    for (const size_t fid : faces) {
        const simplex::Face face = simplex_from_face(fid);
        const size_t opp = face.opposite_vertex(edge).id();
        if (m_face_attribute.at(fid).tags.empty()) {
            log_and_throw_error("No tags in face {}", fid); // for debugging
        }
        cache.faces[opp] = m_face_attribute.at(fid);
    }

    return true;
}

bool ImageSimulationMeshTri::split_edge_after(const Tuple& loc)
{ // input: locs pointing to a list of tets and v_id
    if (!TriMesh::split_edge_after(
            loc)) // note: call from super class, cannot be done with pure virtual classes
        return false;

    const std::vector<Tuple> locs = get_one_ring_tris_for_vertex(loc.switch_vertex(*this));
    const size_t v_id = loc.switch_vertex(*this).vid(*this);

    auto& cache = split_cache.local();

    const size_t v1_id = cache.v1_id;
    const size_t v2_id = cache.v2_id;

    /// check inversion & rounding
    m_vertex_attribute[v_id].m_pos =
        (m_vertex_attribute[v1_id].m_pos + m_vertex_attribute[v2_id].m_pos) / 2;

    for (const Tuple& t : locs) {
        if (is_inverted(t)) {
            return false;
        }
    }

    // update face attributes
    {
        // v1 - v_new
        const auto faces1 = get_incident_fids_for_edge(v1_id, v_id);
        const simplex::Edge edge1(v1_id, v_id);
        for (const size_t fid : faces1) {
            const simplex::Face face = simplex_from_face(fid);
            const size_t opp = face.opposite_vertex(edge1).id();
            m_face_attribute[fid] = cache.faces[opp];
        }
        // v2 - v_new
        const auto faces2 = get_incident_fids_for_edge(v2_id, v_id);
        const simplex::Edge edge2(v2_id, v_id);
        for (const size_t fid : faces2) {
            const simplex::Face face = simplex_from_face(fid);
            const size_t opp = face.opposite_vertex(edge2).id();
            m_face_attribute[fid] = cache.faces[opp];
        }
        assert(faces1.size() + faces2.size() == locs.size());

        const auto [_1, eid1] = tuple_from_edge(edge1.vertices());
        const auto [_2, eid2] = tuple_from_edge(edge2.vertices());

        m_edge_attribute[eid1] = cache.old_e_attrs;
        m_edge_attribute[eid2] = cache.old_e_attrs;
        for (const auto& [vid, _] : cache.faces) {
            const auto [_, eid] = tuple_from_edge({{v_id, vid}});
            m_edge_attribute[eid].reset();
        }
    }

    /// update quality
    for (const Tuple& loc : locs) {
        m_face_attribute[loc.fid(*this)].m_quality = get_quality(loc);
    }

    /// update vertex attribute
    // bbox
    m_vertex_attribute[v_id].on_bbox_faces = wmtk::set_intersection(
        m_vertex_attribute[v1_id].on_bbox_faces,
        m_vertex_attribute[v2_id].on_bbox_faces);
    // surface
    m_vertex_attribute[v_id].m_is_on_surface = cache.old_e_attrs.m_is_surface_fs;

    /// update edge attribute
    for (const auto& [e, e_attr] : cache.changed_edges) {
        auto [_, eid] = tuple_from_edge(e.vertices());
        m_edge_attribute[eid] = e_attr;
    }

    m_vertex_attribute[v_id].partition_id = m_vertex_attribute[v1_id].partition_id;
    m_vertex_attribute[v_id].m_sizing_scalar =
        (m_vertex_attribute[v1_id].m_sizing_scalar + m_vertex_attribute[v2_id].m_sizing_scalar) / 2;

    return true;
}

void ImageSimulationMeshTri::collapse_all_edges(bool is_limit_length)
{
    igl::Timer timer;
    double time;
    timer.start();

    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    {
        const auto all_edges = get_edges();
        logger().info("#E = {}", all_edges.size());
        for (const Tuple& loc : all_edges) {
            // collect all edges. Filtering too long edges happens in `is_weight_up_to_date`
            collect_all_ops.emplace_back("edge_collapse", loc);
            collect_all_ops.emplace_back("edge_collapse", loc.switch_vertex(*this));
        }
    }
    auto collect_failure_ops = tbb::concurrent_vector<std::pair<std::string, Tuple>>();
    std::atomic_int count_success = 0;
    time = timer.getElapsedTime();
    wmtk::logger().info("edge collapse prepare time: {:.4}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = [&count_success](
                                             const ImageSimulationMeshTri& m,
                                             Op op,
                                             const std::vector<Tuple>& newts) {
            count_success++;
            std::vector<std::pair<std::string, Tuple>> op_tups;
            for (const Tuple& t : newts) {
                op_tups.emplace_back(op, t);
                op_tups.emplace_back(op, t.switch_vertex(m));
            }
            return op_tups;
        };
        executor.priority = [](const ImageSimulationMeshTri& m, Op op, const Tuple& t) {
            return -m.get_length2(t);
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [&](const ImageSimulationMeshTri& m,
                                            const std::tuple<double, Op, Tuple>& ele) {
            const auto& VA = m_vertex_attribute;
            auto& [weight, op, tup] = ele;
            const double length = m.get_length2(tup);
            if (length != -weight) {
                return false;
            }
            //
            size_t v1_id = tup.vid(*this);
            size_t v2_id = tup.switch_vertex(*this).vid(*this);
            double sizing_ratio = (VA[v1_id].m_sizing_scalar + VA[v2_id].m_sizing_scalar) / 2;
            if (is_limit_length && length > m_params.collapsing_l2 * sizing_ratio * sizing_ratio)
                return false;
            return true;
        };

        executor.on_fail =
            [&collect_failure_ops](const ImageSimulationMeshTri& m, Op op, const Tuple& t) {
                collect_failure_ops.emplace_back(op, t);
            };
        // Execute!!
        do {
            count_success.store(0, std::memory_order_release);
            wmtk::logger().info("Prepare to collapse {}", collect_all_ops.size());
            executor(*this, collect_all_ops);
            wmtk::logger().info(
                "Collapsed {}, retrying failed {}",
                (int)count_success,
                collect_failure_ops.size());
            collect_all_ops.clear();
            for (const auto& item : collect_failure_ops) {
                collect_all_ops.emplace_back(item);
            }
            collect_failure_ops.clear();
        } while (count_success.load(std::memory_order_acquire) > 0);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = ExecutePass<ImageSimulationMeshTri, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices =
            [](ImageSimulationMeshTri& m, const Tuple& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge collapse operation time parallel: {:.4}s", time);
    } else {
        timer.start();
        auto executor = ExecutePass<ImageSimulationMeshTri, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge collapse operation time serial: {:.4}s", time);
    }
}

bool ImageSimulationMeshTri::collapse_edge_before(const Tuple& loc)
// input is an edge
{
    const auto& VA = m_vertex_attribute;
    auto& cache = collapse_cache.local();

    cache.changed_edges.clear();
    cache.changed_fids.clear();
    cache.changed_energies.clear();
    cache.surface_edges.clear();

    size_t v1_id = loc.vid(*this);
    auto loc1 = switch_vertex(loc);
    size_t v2_id = loc1.vid(*this);

    cache.v1_id = v1_id;
    cache.v2_id = v2_id;

    cache.edge_length = (VA[v1_id].m_pos - VA[v2_id].m_pos).norm();

    ///check if on bbox/surface/boundary
    // bbox
    if (!VA[v1_id].on_bbox_faces.empty()) {
        if (VA[v2_id].on_bbox_faces.size() < VA[v1_id].on_bbox_faces.size()) {
            return false;
        }
        for (int on_bbox : VA[v1_id].on_bbox_faces)
            if (std::find(
                    VA[v2_id].on_bbox_faces.begin(),
                    VA[v2_id].on_bbox_faces.end(),
                    on_bbox) == VA[v2_id].on_bbox_faces.end()) {
                return false;
            }
    }

    // surface
    if (cache.edge_length > 0 && VA[v1_id].m_is_on_surface) {
        if (!VA[v2_id].m_is_on_surface && m_envelope->is_outside(VA[v2_id].m_pos)) {
            return false;
        }
    }

    const auto& n1_locs = get_one_ring_fids_for_vertex(loc);

    cache.changed_fids.reserve(n1_locs.size());
    cache.max_energy = 0;
    for (const size_t& tid : n1_locs) {
        const double q = m_face_attribute.at(tid).m_quality;
        cache.max_energy = std::max(cache.max_energy, q);
        const auto vs = oriented_tri_vids(tid);
        if (vs[0] != v2_id && vs[1] != v2_id && vs[2] != v2_id) {
            cache.changed_fids.emplace_back(tid);
        }
    }

    // pre-compute after-collapse energies
    cache.changed_energies.reserve(cache.changed_fids.size());
    for (const size_t tid : cache.changed_fids) {
        std::array<size_t, 3> vs = oriented_tri_vids(tid);
        for (size_t i = 0; i < 3; ++i) {
            if (vs[i] == v1_id) {
                vs[i] = v2_id;
                break;
            }
        }

        if (is_inverted(vs)) {
            return false;
        }
        double q = get_quality(vs);
        if (q > cache.max_energy) {
            return false;
        }
        cache.changed_energies.emplace_back(q);
    }
    assert(cache.changed_energies.size() == cache.changed_fids.size());

    //
    const auto& n12_locs = get_incident_fids_for_edge(loc);
    for (const size_t& tid : n12_locs) {
        auto vs = oriented_tri_vids(tid);
        std::array<size_t, 2> e_vids = {{v1_id, 0}};
        int cnt = 1;
        // get the vertex that is not v1/v2, i.e., the edge-link vertices.
        for (int j = 0; j < 3; j++) {
            if (vs[j] != v1_id && vs[j] != v2_id) {
                e_vids[cnt] = vs[j];
                cnt++;
            }
        }
        auto [_1, global_eid1] = tuple_from_edge(e_vids);
        auto [_2, global_eid2] = tuple_from_edge({{v2_id, e_vids[1]}});
        auto e_attr = m_edge_attribute.at(global_eid1);
        e_attr.merge(m_edge_attribute.at(global_eid2));
        cache.changed_edges.push_back(std::make_pair(e_attr, e_vids));
    }

    if (VA[v1_id].m_is_on_surface) {
        // this code must check if a face is tagged as surface face
        // only checking the vertices is not enough
        std::vector<std::array<size_t, 2>> fs;
        for (const size_t& tid : n1_locs) {
            const auto vs = oriented_tri_vids(tid);

            int j_v1 = -1;
            auto skip = [&]() {
                for (int j = 0; j < 3; j++) {
                    const size_t vid = vs[j];
                    if (vid == v2_id) {
                        // ignore tets incident to the edge (v1,v2)
                        return true; // v1-v2 definitely not on surface.
                    }
                    if (vid == v1_id) j_v1 = j;
                }
                return false;
            };
            if (skip()) continue;

            for (int k = 0; k < 2; k++) {
                auto va = vs[(j_v1 + 1 + k) % 3];
                if (VA[va].m_is_on_surface) {
                    std::array<size_t, 2> f = {{v1_id, va}};
                    const auto [f_tuple, fid] = tuple_from_edge(f);
                    if (!m_edge_attribute.at(fid).m_is_surface_fs) {
                        // check if this face is actually on the surface
                        continue;
                    }
                    std::sort(f.begin(), f.end());
                    fs.push_back(f);
                }
            }
        }
        wmtk::vector_unique(fs);

        cache.surface_edges.reserve(fs.size());
        for (auto& f : fs) {
            std::replace(f.begin(), f.end(), v1_id, v2_id);
            cache.surface_edges.push_back(f);
        }
    }

    if (m_params.preserve_topology && VA[v1_id].m_is_on_surface && VA[v2_id].m_is_on_surface) {
        // simplified topology preservation check
        size_t eid = loc.eid(*this);
        if (!m_edge_attribute[eid].m_is_surface_fs) {
            return false;
        }
    }

    return true;
}

bool ImageSimulationMeshTri::collapse_edge_after(const Tuple& loc)

{
    auto& VA = m_vertex_attribute;
    auto& cache = collapse_cache.local();
    size_t v1_id = cache.v1_id;
    size_t v2_id = cache.v2_id;

    if (!TriMesh::collapse_edge_after(loc)) {
        return false;
    }

    // surface
    if (cache.edge_length > 0) {
        for (auto& vids : cache.surface_edges) {
            // surface envelope
            bool is_out = m_envelope->is_outside(
                std::array<Vector2d, 2>{VA.at(vids[0]).m_pos, VA.at(vids[1]).m_pos});
            if (is_out) {
                return false;
            }
        }
    }

    //// update attrs
    // tet attr
    for (int i = 0; i < cache.changed_fids.size(); i++) {
        m_face_attribute[cache.changed_fids[i]].m_quality = cache.changed_energies[i];
    }
    // vertex attr
    VA[v2_id].m_is_on_surface = VA.at(v1_id).m_is_on_surface || VA.at(v2_id).m_is_on_surface;

    // no need to update on_bbox_faces
    // face attr
    for (auto& info : cache.changed_edges) {
        auto& f_attr = info.first;
        auto& old_vids = info.second;
        //
        auto [_, global_fid] = tuple_from_edge({{v2_id, old_vids[1]}});
        if (global_fid == -1) {
            return false;
        }
        m_edge_attribute[global_fid] = f_attr;
    }

    return true;
}

size_t ImageSimulationMeshTri::swap_all_edges()
{
    log_and_throw_error("not implemented");
}

bool ImageSimulationMeshTri::swap_edge_before(const Tuple& t)
{
    log_and_throw_error("not implemented");
    return false;
}

bool ImageSimulationMeshTri::swap_edge_after(const Tuple& t)
{
    log_and_throw_error("not implemented");
    return false;
}

void ImageSimulationMeshTri::smooth_all_vertices()
{
    log_and_throw_error("not implemented");
}

bool ImageSimulationMeshTri::smooth_before(const Tuple& t)
{
    log_and_throw_error("not implemented");
    return false;
}

bool ImageSimulationMeshTri::smooth_after(const Tuple& t)
{
    log_and_throw_error("not implemented");
    return false;
}


bool ImageSimulationMeshTri::is_inverted(const std::array<size_t, 3>& vs) const
{
    igl::predicates::exactinit();
    auto res = igl::predicates::orient2d(
        m_vertex_attribute[vs[0]].m_pos,
        m_vertex_attribute[vs[1]].m_pos,
        m_vertex_attribute[vs[2]].m_pos);
    if (res == igl::predicates::Orientation::POSITIVE) {
        return false;
    }
    return true;
}

bool ImageSimulationMeshTri::is_inverted(const Tuple& loc) const
{
    return is_inverted(oriented_tri_vids(loc));
}

double ImageSimulationMeshTri::get_quality(const std::array<size_t, 3>& vs) const
{
    std::array<Vector2d, 3> ps;
    for (size_t k = 0; k < 3; k++) {
        ps[k] = m_vertex_attribute[vs[k]].m_pos;
    }
    double energy = -1.;
    {
        std::array<double, 6> T;
        for (size_t k = 0; k < 3; k++)
            for (size_t j = 0; j < 2; j++) {
                T[k * 2 + j] = ps[k][j];
            }
        energy = AMIPS2D_energy(T);
    }
    if (std::isinf(energy) || std::isnan(energy)) {
        return MAX_ENERGY;
    }
    return energy;
}

double ImageSimulationMeshTri::get_quality(const Tuple& loc) const
{
    return get_quality(oriented_tri_vids(loc));
}

bool ImageSimulationMeshTri::is_edge_on_surface(const Tuple& loc) const
{
    const auto vs = get_edge_vids(loc);
    if (!m_vertex_attribute.at(vs[0]).m_is_on_surface ||
        !m_vertex_attribute.at(vs[1]).m_is_on_surface) {
        return false;
    }

    const size_t eid = loc.eid(*this);
    return m_edge_attribute[eid].m_is_surface_fs;
}
bool ImageSimulationMeshTri::is_edge_on_bbox(const Tuple& loc) const
{
    const auto vs = get_edge_vids(loc);
    if (m_vertex_attribute.at(vs[0]).on_bbox_faces.empty() ||
        m_vertex_attribute.at(vs[1]).on_bbox_faces.empty()) {
        return false;
    }

    const size_t eid = loc.eid(*this);
    return m_edge_attribute[eid].m_is_bbox_fs >= 0;
}

void ImageSimulationMeshTri::mesh_improvement(int max_its)
{
    ////preprocessing
    partition_mesh_morton();

    // write_vtu(fmt::format("debug_{}", m_debug_print_counter++));

    wmtk::logger().info("========it pre========");
    local_operations({{0, 1, 0, 0}}, false);

    ////operation loops
    bool is_hit_min_edge_length = false;
    const int M = 2;
    int m = 0;
    double pre_max_energy = 0., pre_avg_energy = 0.;
    for (int it = 0; it < max_its; it++) {
        ///ops
        wmtk::logger().info("\n========it {}========", it);
        auto [max_energy, avg_energy] = local_operations({{1, 1, 1, 1}});

        ///energy check
        wmtk::logger().info("max energy {} stop {}", max_energy, m_params.stop_energy);
        if (max_energy < m_params.stop_energy) break;
        consolidate_mesh();

        wmtk::logger().info("#V =  {}, #T = {}", vert_capacity(), tri_capacity());

        ///sizing field
        if (it > 0 && pre_max_energy - max_energy < 5e-1 &&
            (pre_avg_energy - avg_energy) / avg_energy < 0.1) {
            m++;
            if (m == M) {
                wmtk::logger().info(">>>>adjust_sizing_field...");
                is_hit_min_edge_length = adjust_sizing_field_serial(max_energy);
                // is_hit_min_edge_length = adjust_sizing_field(max_energy);
                wmtk::logger().info(">>>>adjust_sizing_field finished...");
                m = 0;
            }
        } else {
            m = 0;
            pre_max_energy = max_energy;
            pre_avg_energy = avg_energy;
        }
        if (is_hit_min_edge_length) {
            // todo: maybe to do sth
        }
    }

    wmtk::logger().info("========it post========");
    local_operations({{0, 1, 0, 0}});
}

std::tuple<double, double> ImageSimulationMeshTri::local_operations(
    const std::array<int, 4>& ops,
    bool collapse_limit_length)
{
    igl::Timer timer;

    std::tuple<double, double> energy;

    auto sanity_checks = [this]() {
        if (!m_params.perform_sanity_checks) {
            return;
        }
        logger().info("Perform sanity checks...");
        const auto faces = get_edges_by_condition([](auto& f) { return f.m_is_surface_fs; });
        for (const auto& verts : faces) {
            const auto& p0 = m_vertex_attribute[verts[0]].m_pos;
            const auto& p1 = m_vertex_attribute[verts[1]].m_pos;
            if (m_envelope->is_outside(std::array<Vector2d, 2>{p0, p1})) {
                logger().error("Edge {} is outside!", verts);
            }
        }

        // check for inverted faces
        for (const Tuple& t : get_faces()) {
            if (!is_inverted(t)) {
                continue;
            }
            const auto vs = oriented_tri_vids(t);
            logger().error("Face {} is inverted! Vertices = {}", t.fid(*this), vs);
        }
        logger().info("Sanity checks done.");
    };

    sanity_checks();

    timer.start();
    for (int i = 0; i < ops.size(); i++) {
        if (i == 0) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==splitting {}==", n);
                split_all_edges();
                wmtk::logger().info(
                    "#V = {}, #F = {} after split",
                    get_vertices().size(),
                    get_faces().size());
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", debug_print_counter++));
            }
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("split max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        } else if (i == 1) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==collapsing {}==", n);
                collapse_all_edges(collapse_limit_length);
                wmtk::logger().info(
                    "#V = {}, #F = {} after collapse",
                    get_vertices().size(),
                    get_faces().size());
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", debug_print_counter++));
            }
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("collapse max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        } else if (i == 2) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==swapping {}==", n);
                int cnt_success = swap_all_edges();
                if (cnt_success == 0) {
                    break;
                }
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", debug_print_counter++));
            }
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("swap max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        } else if (i == 3) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==smoothing {}==", n);
                smooth_all_vertices();
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", debug_print_counter++));
            }
            auto [max_energy, avg_energy] = get_max_avg_energy();
            wmtk::logger().info("smooth max energy = {:.6} avg = {:.6}", max_energy, avg_energy);
            sanity_checks();
        }
    }
    energy = get_max_avg_energy();
    wmtk::logger().info("max energy = {:.6}", std::get<0>(energy));
    wmtk::logger().info("avg energy = {:.6}", std::get<1>(energy));
    wmtk::logger().info("time = {:.4}s", timer.getElapsedTimeInSec());


    return energy;
}

std::tuple<double, double> ImageSimulationMeshTri::get_max_avg_energy()
{
    double max_energy = -1.;
    double avg_energy = 0.;
    auto cnt = 0;

    for (int i = 0; i < tri_capacity(); i++) {
        const Tuple tup = tuple_from_tri(i);
        if (!tup.is_valid(*this)) {
            continue;
        }
        const double q = m_face_attribute[tup.fid(*this)].m_quality;
        max_energy = std::max(max_energy, q);
        avg_energy += q;
        cnt++;
    }

    avg_energy /= cnt;

    return std::make_tuple(max_energy, avg_energy);
}


} // namespace wmtk::components::image_simulation::tri