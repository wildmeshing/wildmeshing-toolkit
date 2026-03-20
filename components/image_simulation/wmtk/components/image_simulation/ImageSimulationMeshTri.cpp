#include "ImageSimulationMeshTri.hpp"
#include "ConnectedComponentAnnotationHelper.cpp"

#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
#include <igl/predicates/predicates.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <map>
#include <paraviewo/VTUWriter.hpp>
#include <set>
#include <unordered_map>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/envelope/KNN.hpp>
#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/DirichletEnergy.hpp>
#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/EnvelopeEnergy.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include <wmtk/utils/io.hpp>


namespace {
static int debug_print_counter = 0;
}

namespace wmtk::components::image_simulation::tri {

auto renew = [](const ImageSimulationMeshTri& m, auto op, auto& tris) {
    using Tuple = TriMesh::Tuple;
    std::vector<Tuple> edges;
    for (const auto& t : tris) {
        for (auto j = 0; j < 3; j++) {
            edges.push_back(m.tuple_from_edge(t.fid(m), j));
        }
    }
    wmtk::unique_edge_tuples(m, edges);

    std::vector<std::pair<std::string, Tuple>> optup;
    optup.reserve(edges.size());
    for (const Tuple& e : edges) {
        optup.emplace_back(op, e);
    }
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
            for (int i = 0; i < 2; i++) {
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
                    list_v[i].morton =
                        Resorting::MortonCode64(int(V[i][0] * multi), int(V[i][1] * multi), 0);
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
    for (size_t i = 0; i < (size_t)T_tags.rows(); ++i) {
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

void ImageSimulationMeshTri::init_envelope(const MatrixXd& V, const MatrixXi& E)
{
    if (m_envelope) {
        log_and_throw_error("Envelope was already initialized once.");
    }
    assert(m_V_envelope.empty() && m_E_envelope.empty());
    assert(V.size() != 0 && E.size() != 0);
    assert(V.cols() == 2); // vertices must be in 2D
    assert(E.cols() == 2); // envelope must be edges


    m_V_envelope.resize(V.rows());
    for (size_t i = 0; i < m_V_envelope.size(); ++i) {
        m_V_envelope[i] = V.row(i);
    }
    m_E_envelope.resize(E.rows());
    for (size_t i = 0; i < m_E_envelope.size(); ++i) {
        m_E_envelope[i] = E.row(i);
    }

    m_envelope = std::make_shared<SampleEnvelope>();
    m_envelope->init(m_V_envelope, m_E_envelope, m_envelope_eps);
}

bool ImageSimulationMeshTri::adjust_sizing_field_serial(double max_energy)

{
    wmtk::logger().info("#V {}, #F {}", vert_capacity(), tri_capacity());

    const double stop_filter_energy = m_params.stop_energy * 0.8;
    double filter_energy = std::max(max_energy / 100, stop_filter_energy);
    filter_energy = std::min(filter_energy, 100.);

    const auto recover_scalar = 1.5;
    const auto refine_scalar = 0.5;
    const auto min_refine_scalar = m_params.l_min / m_params.l;

    // outputs scale_multipliers
    std::vector<double> scale_multipliers(vert_capacity(), recover_scalar);

    std::vector<Vector3d> pts;
    std::queue<size_t> v_queue;

    for (int i = 0; i < tri_capacity(); i++) {
        const Tuple t = tuple_from_tri(i);
        if (!t.is_valid(*this)) {
            continue;
        }
        const size_t fid = t.fid(*this);
        if (m_face_attribute.at(fid).m_quality < filter_energy) {
            continue;
        }
        const auto vs = oriented_tri_vids(t);
        Vector2d c(0, 0); // center
        for (int j = 0; j < 3; j++) {
            c += m_vertex_attribute.at(vs[j]).m_pos;
            v_queue.emplace(vs[j]);
        }
        c /= 3;
        pts.emplace_back(Vector3d(c[0], c[1], 0));
    }

    wmtk::logger().info("filter energy {} Low Quality Tets {}", filter_energy, pts.size());

    const double R = m_params.l * 1.8;

    int sum = 0;
    int adjcnt = 0;

    std::vector<bool> visited(vert_capacity(), false);

    KNN knn(pts);

    std::vector<size_t> cache_one_ring;
    // size_t vid;
    while (!v_queue.empty()) {
        sum++;
        const size_t vid = v_queue.front();
        v_queue.pop();
        if (visited[vid]) continue;
        visited[vid] = true;
        adjcnt++;

        const auto& pos_v = m_vertex_attribute.at(vid).m_pos;
        const Vector3d p(pos_v[0], pos_v[1], 0);
        double sq_dist = 0.;
        uint32_t idx;
        knn.nearest_neighbor(p, idx, sq_dist);
        const double dist = std::sqrt(sq_dist);

        if (dist > R) { // outside R-ball, unmark.
            continue;
        }

        scale_multipliers[vid] = std::min(
            scale_multipliers[vid],
            dist / R * (1 - refine_scalar) + refine_scalar); // linear interpolate

        get_one_ring_vids_for_vertex_duplicate(vid, cache_one_ring);
        for (size_t n_vid : cache_one_ring) {
            if (visited[n_vid]) {
                continue;
            }
            v_queue.push(n_vid);
        }
    }

    logger().info("sum = {}; adjacent = {}", sum, adjcnt);

    std::atomic_bool is_hit_min_edge_length = false;

    for (int i = 0; i < vert_capacity(); i++) {
        const Tuple v = tuple_from_vertex(i);
        if (!v.is_valid(*this)) {
            continue;
        }
        const size_t vid = v.vid(*this);
        auto& v_attr = m_vertex_attribute[vid];

        auto new_scale = v_attr.m_sizing_scalar * scale_multipliers[vid];
        if (new_scale > 1) {
            v_attr.m_sizing_scalar = 1;
        } else if (new_scale < min_refine_scalar) {
            is_hit_min_edge_length = true;
            v_attr.m_sizing_scalar = min_refine_scalar;
        } else {
            v_attr.m_sizing_scalar = new_scale;
        }
    }

    return is_hit_min_edge_length.load();
}

void ImageSimulationMeshTri::write_msh(std::string file)
{
    consolidate_mesh();

    wmtk::MshData msh;

    const auto& vtx = get_vertices();
    msh.add_face_vertices(vtx.size(), [&](size_t k) {
        auto i = vtx[k].vid(*this);
        Vector2d p2 = m_vertex_attribute[i].m_pos;
        return Vector3d(p2[0], p2[1], 0);
    });

    const auto faces = get_faces();
    msh.add_faces(faces.size(), [&](size_t k) {
        const size_t fid = faces[k].fid(*this);
        auto vs = oriented_tri_vertices(faces[k]);
        std::array<size_t, 3> data;
        for (size_t j = 0; j < 3; j++) {
            data[j] = vs[j].vid(*this);
            assert(data[j] < vtx.size());
        }
        return data;
    });

    msh.add_face_vertex_attribute<1>("sizing_scalar", [&](size_t i) {
        return m_vertex_attribute[i].m_sizing_scalar;
    });
    msh.add_face_attribute<1>("quality", [&](size_t i) { return m_face_attribute[i].m_quality; });

    for (size_t j = 0; j < m_tags_count; ++j) {
        msh.add_face_attribute<1>(fmt::format("tag_{}", j), [&](size_t i) {
            return m_face_attribute[i].tags[j];
        });
    }

    msh.add_physical_group("ImageVolume");

    msh.add_edge_vertices(m_V_envelope.size(), [this](size_t k) {
        return Vector3d(m_V_envelope[k][0], m_V_envelope[k][1], 0);
    });
    msh.add_edges(m_E_envelope.size(), [this](size_t k) { return m_E_envelope[k]; });
    msh.add_physical_group("EnvelopeSurface");

    logger().info("Write {}", file);
    msh.save(file, true);
}

void ImageSimulationMeshTri::write_vtu(const std::string& path) const
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
        for (size_t j = 0; j < 3; j++) {
            F(index, j) = (int)vs[j];
        }
        ++index;
    }

    for (size_t i = 0; i < edges.size(); ++i) {
        for (size_t j = 0; j < 2; ++j) {
            E(i, j) = (int)edges[i][j];
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
    std::function<bool(const EdgeAttributes&)> cond) const
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
    auto& p = m_vertex_attribute[v_id].m_pos;
    p = (m_vertex_attribute[v1_id].m_pos + m_vertex_attribute[v2_id].m_pos) / 2;

    for (const Tuple& t : locs) {
        if (is_inverted(t)) {
            return false;
        }
    }

    // If a Voronoi split function is set, binary-search vmid onto its zero-crossing.
    // p0 stays on the negative side, p1 on the positive side.
    if (m_voronoi_split_fn) {
        Vector2d p0 = m_vertex_attribute[v1_id].m_pos;
        Vector2d p1 = m_vertex_attribute[v2_id].m_pos;
        if (m_voronoi_split_fn(p0) >= 0) std::swap(p0, p1); // ensure p0 is negative side
        for (int i = 0; i < 20; ++i) {
            p = 0.5 * (p0 + p1);
            bool inv = false;
            for (const Tuple& t : locs) {
                if (is_inverted(t)) {
                    inv = true;
                    break;
                }
            }
            if (inv || (p1 - p0).squaredNorm() < 1e-20) break;
            if (m_voronoi_split_fn(p) < 0)
                p0 = p;
            else
                p1 = p;
        }
        // final inversion guard: revert to midpoint if needed
        bool inv = false;
        for (const Tuple& t : locs) {
            if (is_inverted(t)) {
                inv = true;
                logger().warn(
                    "Voronoi split resulted in inversion, reverting to midpoint. Iteration: {}",
                    debug_print_counter++);
                break;
            }
        }
        if (inv) p = (m_vertex_attribute[v1_id].m_pos + m_vertex_attribute[v2_id].m_pos) / 2;
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
            const auto [_tup, eid] = tuple_from_edge({{v_id, vid}});
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
    std::vector<std::pair<std::string, Tuple>> all_ops;

    auto setup_and_execute = [&](auto& executor) {
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

        // Execute!!
        do {
            all_ops.clear();
            const auto all_edges = get_edges();
            logger().info("#E = {}", all_edges.size());
            for (const Tuple& loc : all_edges) {
                // collect all edges. Filtering too long edges happens in `is_weight_up_to_date`
                all_ops.emplace_back("edge_collapse", loc);
                all_ops.emplace_back("edge_collapse", loc.switch_vertex(*this));
            }
            executor(*this, all_ops);
            logger().info(
                "success: {}, failed: {}",
                executor.get_cnt_success(),
                executor.get_cnt_fail());
        } while (executor.get_cnt_success() > 0);
    };

    igl::Timer timer;
    timer.start();
    if (NUM_THREADS > 0) {
        auto executor = ExecutePass<ImageSimulationMeshTri, ExecutionPolicy::kPartition>();
        executor.lock_vertices =
            [](ImageSimulationMeshTri& m, const Tuple& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        wmtk::logger().info("edge collapse time parallel: {:.4}s", timer.getElapsedTimeInSec());
    } else {
        auto executor = ExecutePass<ImageSimulationMeshTri, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        wmtk::logger().info("edge collapse time serial: {:.4}s", timer.getElapsedTimeInSec());
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
        // if (!VA[v2_id].m_is_on_surface && m_envelope->is_outside(VA[v2_id].m_pos)) {
        //     return false;
        // }
        if (!VA[v2_id].m_is_on_surface) {
            return false; // do not collapse away from surface
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
        if (q > m_params.stop_energy && q > cache.max_energy) {
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

    if (m_params.preserve_topology) {
        const bool v1_surf = VA[v1_id].m_is_on_surface;
        const bool v2_surf = VA[v2_id].m_is_on_surface;
        const bool v1_bbox = !VA[v1_id].on_bbox_faces.empty();
        const bool v2_bbox = !VA[v2_id].on_bbox_faces.empty();

        if ((v1_surf || v1_bbox) && (v2_surf || v2_bbox)) {
            if (!substructure_link_condition(loc)) {
                return false;
            }
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
            const Vector2d a = VA.at(vids[0]).m_pos;
            const Vector2d b = VA.at(vids[1]).m_pos;
            // surface envelope
            bool is_out = m_envelope->is_outside(std::array<Vector2d, 2>{a, b});
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
    std::vector<std::pair<std::string, Tuple>> collect_all_ops;
    {
        igl::Timer timer;
        timer.start();
        const auto edges = get_edges();
        collect_all_ops.reserve(edges.size());
        for (const Tuple& t : edges) {
            collect_all_ops.emplace_back("edge_swap", t);
        }
        timer.stop();
        logger().info("edge collapse prepare time: {:.4}s", timer.getElapsedTimeInSec());
    }
    logger().info("#E = {}", collect_all_ops.size());

    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = renew;
        executor.num_threads = NUM_THREADS;
        executor.priority = [](const ImageSimulationMeshTri& m, std::string op, const Tuple& e) {
            return m.swap_weight(e);
        };
        executor.should_renew = [](auto val) { return (val > 0); };
        executor.is_weight_up_to_date = [](const ImageSimulationMeshTri& m, auto& ele) {
            auto& [val, _, e] = ele;
            const double w = m.swap_weight(e);
            return (w > 1e-5) && ((w - val) * (w - val) < 1e-8);
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<ImageSimulationMeshTri, ExecutionPolicy::kPartition>();
        executor.lock_vertices = edge_locker;
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<ImageSimulationMeshTri, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }

    return true;
}

double ImageSimulationMeshTri::swap_weight(const Tuple& t) const
{
    const SmartTuple tt(*this, t);
    const auto t_opp = tt.switch_face();
    if (!t_opp) {
        return std::numeric_limits<double>::lowest();
    }

    if (is_edge_on_surface(t)) {
        return std::numeric_limits<double>::lowest();
    }

    const size_t v0 = tt.vid();
    const size_t v1 = tt.switch_vertex().vid();
    const size_t v2 = tt.switch_edge().switch_vertex().vid();
    const size_t v3 = t_opp.value().switch_edge().switch_vertex().vid();

    // before swap
    const double q012 = get_quality({{v0, v1, v2}});
    const double q031 = get_quality({{v0, v3, v1}});
    // after swap
    const double q032 = get_quality({{v0, v3, v2}});
    const double q231 = get_quality({{v2, v3, v1}});

    const double q_before = std::max(q012, q031);
    const double q_after = std::max(q032, q231);

    return q_before - q_after;
}

bool ImageSimulationMeshTri::swap_edge_before(const Tuple& t)
{
    if (is_edge_on_surface(t)) {
        return false;
    }

    const auto& FA = m_face_attribute;
    auto& cache = swap_cache.local();
    cache.changed_edges.clear();

    const auto incident_faces = get_incident_fids_for_edge(t);

    cache.face_tags = FA[incident_faces[0]].tags;

    double max_energy = -1.0;
    for (const size_t fid : incident_faces) {
        max_energy = std::max(FA[fid].m_quality, max_energy);
        if (FA[fid].tags != cache.face_tags) {
            log_and_throw_error("not all tets have the same tag"); // for debugging
        }
    }
    cache.max_energy = max_energy;

    // cache edges
    simplex::Edge edge = simplex_from_edge(t);
    for (const size_t fid : incident_faces) {
        for (int j = 0; j < 3; j++) {
            const Tuple tup = tuple_from_edge(fid, j);
            simplex::Edge e = simplex_from_edge(tup);
            if (e == edge) {
                continue;
            }
            cache.changed_edges.try_emplace(e, m_edge_attribute[tup.eid(*this)]);
        }
    }

    return true;
}

bool ImageSimulationMeshTri::swap_edge_after(const Tuple& t)
{
    auto& cache = swap_cache.local();
    const auto incident_faces = get_incident_fids_for_edge(t);

    auto& FA = m_face_attribute;

    double max_energy = -1.0;
    for (const size_t fid : incident_faces) {
        if (is_inverted(fid)) {
            return false;
        }
        double q = get_quality(fid);
        FA[fid].m_quality = q;
        max_energy = std::max(q, max_energy);

        FA[fid].tags = cache.face_tags;
    }
    if (max_energy >= cache.max_energy) {
        return false;
    }

    // cached edges
    for (const auto& [e, e_attrs] : cache.changed_edges) {
        const auto [_, eid] = tuple_from_edge(e.vertices());
        m_edge_attribute[eid] = e_attrs;
    }
    m_edge_attribute[t.eid(*this)].reset();

    return true;
}

void ImageSimulationMeshTri::smooth_all_vertices()
{
    assert(m_solver);

    // build mass-matrix
    {
        const auto vs = get_vertices();
        m_surface_mass.resize(vert_capacity());
        m_surface_stiffness.resize(vert_capacity());
        for (const Tuple& t : vs) {
            const size_t vid = t.vid(*this);
            if (!m_vertex_attribute.at(vid).m_is_on_surface) {
                continue;
            }
            const auto es = get_order1_edges_for_vertex(vid);
            if (es.size() != 2) {
                continue;
            }
            auto& M = m_surface_mass[vid];
            auto& L_w = m_surface_stiffness[vid];

            std::array<Vector2d, 3> pts;
            pts[0] = m_vertex_attribute.at(vid).m_pos;

            for (size_t i = 0; i < 2; ++i) {
                const auto& vs = es.edges()[i].vertices();
                size_t neighbor_id = vs[0] != vid ? vs[0] : vs[1];
                pts[i + 1] = m_vertex_attribute.at(neighbor_id).m_pos;
            }

            optimization::BiharmonicEnergy2D::local_mass_and_stiffness(pts, M, L_w);
            // optimization::SmoothingEnergy2D::uniform_mass_and_stiffness(pts, M, L_w);
        }
    }
    // init barrier energy
    {
        logger().info("Build barrier energy");
        MatrixXd V;
        MatrixXi E;
        // gather all surface edges
        std::vector<Vector2d> surf_points;
        m_global_to_local_vid_map.resize(vert_capacity());
        for (size_t i = 0; i < vert_capacity(); ++i) {
            const Tuple v = tuple_from_vertex(i);
            if (!v.is_valid(*this)) {
                continue;
            }
            const size_t vid = v.vid(*this);
            if (!m_vertex_attribute.at(vid).m_is_on_surface) {
                continue;
            }
            surf_points.push_back(m_vertex_attribute.at(vid).m_pos);
            m_global_to_local_vid_map[vid] = surf_points.size() - 1;
        }

        V.resize(surf_points.size(), 2);
        for (size_t i = 0; i < surf_points.size(); ++i) {
            V.row(i) = surf_points[i];
        }

        const auto surf_edges = get_edges_by_condition([](auto& f) { return f.m_is_surface_fs; });
        E.resize(surf_edges.size(), 2);
        for (size_t i = 0; i < surf_edges.size(); ++i) {
            const size_t v0 = m_global_to_local_vid_map[surf_edges[i][0]];
            const size_t v1 = m_global_to_local_vid_map[surf_edges[i][1]];
            E.row(i) = Vector2i(v0, v1);
        }

        const double dhat = 1.0; // TODO should not be hard-coded!!!
        m_barrier_energy = std::make_shared<optimization::BarrierEnergy2D>(V, E, 0, dhat);
        logger().info("Finished building barrier energy.");
    }

    igl::Timer timer;
    timer.start();
    std::vector<std::pair<std::string, Tuple>> collect_all_ops;
    for (const Tuple& t : get_vertices()) {
        collect_all_ops.emplace_back("vertex_smooth", t);
    }
    logger().info("vertex smoothing prepare time: {:.4}s", timer.getElapsedTimeInSec());
    logger().info("#V = {}", collect_all_ops.size());
    if (NUM_THREADS > 0) {
        timer.start();
        ExecutePass<ImageSimulationMeshTri, ExecutionPolicy::kPartition> executor;
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_vertex_mutex_one_ring(e, task_id);
        };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
        logger().info("vertex smoothing time parallel: {:.4}s", timer.getElapsedTimeInSec());
    } else {
        timer.start();
        ExecutePass<ImageSimulationMeshTri, ExecutionPolicy::kSeq> executor;
        executor(*this, collect_all_ops);
        logger().info("vertex smoothing time serial: {:.4}s", timer.getElapsedTimeInSec());
    }
}

bool ImageSimulationMeshTri::smooth_before(const Tuple& t)
{
    const size_t vid = t.vid(*this);
    if (!m_vertex_attribute.at(vid).on_bbox_faces.empty()) {
        return false;
    }

    return true;
}

bool ImageSimulationMeshTri::smooth_after(const Tuple& t)
{
    // Newton iterations are encapsulated here.
    logger().trace("Newton iteration for vertex smoothing.");
    const size_t vid = t.vid(*this);

    const auto& VA = m_vertex_attribute;

    const auto locs = get_one_ring_fids_for_vertex(t);
    assert(locs.size() > 0);

    double max_quality = 0.;
    for (const size_t fid : locs) {
        max_quality = std::max(max_quality, m_face_attribute[fid].m_quality);
    }

    std::vector<std::array<double, 6>> assembles;
    assembles.reserve(locs.size());

    for (const size_t fid : locs) {
        if (is_inverted(fid)) {
            log_and_throw_error("Inverted face before smoothing!");
        }
        std::array<size_t, 3> local_verts = oriented_tri_vids(fid);
        {
            size_t v_loc = 0;
            for (size_t i = 0; i < 3; ++i) {
                if (local_verts[i] == vid) {
                    v_loc = i;
                    break;
                }
            }
            std::array<size_t, 3> buf = local_verts;
            local_verts[0] = buf[v_loc];
            local_verts[1] = buf[(v_loc + 1) % 3];
            local_verts[2] = buf[(v_loc + 2) % 3];
        }

        std::array<double, 6> T;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 2; j++) {
                T[i * 2 + j] = VA[local_verts[i]].m_pos[j];
            }
        }
        assembles.push_back(T);
    }

    const Vector2d old_pos = VA[vid].m_pos;

    // call to polysolve
    std::shared_ptr<optimization::EnergySum::Problem> total_energy;
    auto amips_energy = std::make_shared<optimization::AMIPSEnergy2D>(assembles);
    total_energy = amips_energy;
    //{
    //    auto x = amips_energy->initial_position();
    //    try {
    //        m_solver->minimize(*amips_energy, x);
    //    } catch (const std::exception& e) {
    //        // polysolve might throw errors that we want to ignore (e.g., line search failed)
    //    }
    //    m_vertex_attribute[vid].m_pos = x;
    //}

    // m_vertex_attribute[vid].m_pos =
    //     newton_method_from_stack(assembles, AMIPS2D_energy, AMIPS2D_jacobian, AMIPS2D_hessian);

    wmtk::logger().trace(
        "old pos {} -> new pos {}",
        old_pos.transpose(),
        VA[vid].m_pos.transpose());

    std::array<Vector2d, 3> surface_pts;
    if (VA[vid].m_is_on_surface) {
        const auto es = get_order1_edges_for_vertex(vid);
        if (es.size() != 2) {
            return false; // can only smooth vertices with 2 neighbors
        }

        surface_pts[0] = m_vertex_attribute.at(vid).m_pos;

        for (size_t i = 0; i < 2; ++i) {
            const auto& vs = es.edges()[i].vertices();
            size_t neighbor_id = vs[0] != vid ? vs[0] : vs[1];
            surface_pts[i + 1] = m_vertex_attribute.at(neighbor_id).m_pos;
        }

        // project to surface
        //{
        //    assert(m_envelope->initialized());
        //    Vector2d project;
        //    m_envelope->nearest_point(VA[vid].m_pos, project);
        //
        //    m_vertex_attribute[vid].m_pos = project;
        //}

        const auto& M = m_surface_mass[vid];
        const auto& L_w = m_surface_stiffness[vid];

        auto smooth_energy =
            std::make_shared<optimization::BiharmonicEnergy2D>(surface_pts, M, L_w);
        auto envelope_energy =
            std::make_shared<optimization::EnvelopeEnergy2D>(m_envelope, surface_pts);
        auto energy_sum = std::make_shared<optimization::EnergySum>();
        energy_sum->add_energy(amips_energy);
        energy_sum->add_energy(smooth_energy, 1e2);
        energy_sum->add_energy(envelope_energy, 1e2 * M);

        m_barrier_energy->replace_vid(m_global_to_local_vid_map[vid]);
        energy_sum->add_energy(m_barrier_energy, 1e4);

        total_energy = energy_sum;
    }

    // solve
    {
        auto x = amips_energy->initial_position();
        try {
            m_solver->minimize(*total_energy, x);
        } catch (const std::exception&) {
            // polysolve might throw errors that we want to ignore (e.g., line search failed)
        }
        m_vertex_attribute[vid].m_pos = x;
    }

    // check surface containment
    if (VA[vid].m_is_on_surface) {
        for (size_t i = 0; i < 2; ++i) {
            std::array<Eigen::Vector2d, 2> edge;
            edge[0] = VA[vid].m_pos;
            edge[1] = surface_pts[i + 1];
            if (m_envelope->is_outside(edge)) {
                m_barrier_energy->V().row(m_global_to_local_vid_map[vid]) = old_pos;
                return false;
            }
        }
    }

    // quality (only check if not on surface)
    auto max_after_quality = 0.;
    for (const size_t fid : locs) {
        if (is_inverted(fid)) {
            if (VA[vid].m_is_on_surface) {
                m_barrier_energy->V().row(m_global_to_local_vid_map[vid]) = old_pos;
            }
            return false;
        }
        const double q = get_quality(fid);
        m_face_attribute[fid].m_quality = q;
        max_after_quality = std::max(max_after_quality, q);
    }
    if (!VA[vid].m_is_on_surface) {
        if (max_after_quality > max_quality) {
            return false;
        }
    }

    return true;
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

bool ImageSimulationMeshTri::is_inverted(const size_t fid) const
{
    return is_inverted(oriented_tri_vids(fid));
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
    if (std::isinf(energy) || std::isnan(energy) || energy < 2 - 1e-3) {
        return MAX_ENERGY;
    }
    return energy;
}

double ImageSimulationMeshTri::get_quality(const Tuple& loc) const
{
    return get_quality(oriented_tri_vids(loc));
}

double ImageSimulationMeshTri::get_quality(const size_t fid) const
{
    return get_quality(oriented_tri_vids(fid));
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
bool ImageSimulationMeshTri::is_edge_on_surface(const std::array<size_t, 2>& vids) const
{
    if (!m_vertex_attribute.at(vids[0]).m_is_on_surface ||
        !m_vertex_attribute.at(vids[1]).m_is_on_surface) {
        return false;
    }

    const auto [_, eid] = tuple_from_edge(vids);
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

bool ImageSimulationMeshTri::is_edge_on_bbox(const std::array<size_t, 2>& vids) const
{
    if (m_vertex_attribute.at(vids[0]).on_bbox_faces.empty() ||
        m_vertex_attribute.at(vids[1]).on_bbox_faces.empty()) {
        return false;
    }
    const auto [_, eid] = tuple_from_edge(vids);
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
        if (max_energy < m_params.stop_energy) {
            break;
        }
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
                size_t cnt_success = swap_all_edges();
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
                if (m_params.debug_output) {
                    write_vtu(fmt::format("debug_{}", debug_print_counter++));
                }
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

void ImageSimulationMeshTri::fill_holes_topo(
    const std::vector<int64_t>& fill_holes_tags,
    double threshold)
{
    if (m_tags_count == 0) {
        logger().warn("fill_holes_topo: no tags, skipping");
        return;
    }

    // -- Step 1: compute all connected components with their surrounding component ids
    auto components = compute_connected_components();

    // -- Step 2: for each fill_tag, fill all enclosed components
    bool any_filled = false;
    for (const int64_t fill_tag : fill_holes_tags) {
        std::vector<std::vector<size_t>> hole_clusters;
        std::unordered_set<int64_t> tags{fill_tag};
        extract_hole_clusters(components, tags, hole_clusters, threshold);

        for (const auto& hole_cluster : hole_clusters) {
            // Collect the fill-tag component indices that surround this cluster
            std::unordered_set<size_t> engulfing_comp_ids;
            for (const size_t ci : hole_cluster) {
                for (const size_t sid : components[ci].surrounding_comp_ids) {
                    if (!components[sid].faces.empty() && components[sid].tag == fill_tag) {
                        engulfing_comp_ids.insert(sid);
                    }
                }
            }
            engulf_components(components, hole_cluster, engulfing_comp_ids);
            any_filled = true;
        }
        if (hole_clusters.empty()) {
            logger().info(
                "fill_holes_topo: no enclosed components found for fill_tag {}",
                fill_tag);
        }
    }

    if (!any_filled) return;

    // -- Step 3: recompute m_is_surface_fs and m_is_on_surface
    recompute_surface_info();
}
void ImageSimulationMeshTri::keep_largest_connected_component(const std::vector<int64_t>& lcc_tags)
{
    auto components = compute_connected_components();
    if (components.empty()) {
        logger().warn("keep_largest_connected_component: no components found, skipping");
        return;
    }
    // For each tag in lcc_tags, find the largest component with that tag and mark all other
    // components with that tag for removal
    for (const int64_t lcc_tag : lcc_tags) {
        int largest_comp_id = -1;
        double largest_area = -1.0;
        for (size_t i = 0; i < components.size(); ++i) {
            if (components[i].faces.empty() || components[i].tag != lcc_tag) continue;
            if (components[i].area > largest_area) {
                largest_area = components[i].area;
                largest_comp_id = i;
            }
        }
        if (largest_comp_id == -1) {
            logger().warn(
                "keep_largest_connected_component: no component found for tag {}, skipping",
                lcc_tag);
            continue;
        }
        for (size_t i = 0; i < components.size(); ++i) {
            if (components[i].faces.empty() || components[i].tag != lcc_tag ||
                (int)i == largest_comp_id)
                continue;
            // Copy surrounding_comp_ids — engulf_components will reset components[i]
            std::unordered_set<size_t> surr = components[i].surrounding_comp_ids;
            engulf_components(components, std::vector<size_t>{i}, surr);
        }
    }

    recompute_surface_info();
}

void ImageSimulationMeshTri::tight_seal_topo(
    const std::vector<std::unordered_set<int64_t>>& tight_seal_tag_sets,
    double threshold)
{
    auto components = compute_connected_components();

    for (const auto& tag_set : tight_seal_tag_sets) {
        std::vector<std::vector<size_t>> hole_clusters;
        std::unordered_set<int64_t> tags_copy(tag_set.begin(), tag_set.end());
        extract_hole_clusters(components, tags_copy, hole_clusters, threshold);
        for (const auto& hole_cluster : hole_clusters) {
            // Collect component indices in tag_set that surround this cluster
            std::unordered_set<size_t> engulfing_comp_ids;
            std::unordered_set<int64_t> surrounding_tags;
            for (const size_t ci : hole_cluster) {
                for (const size_t sid : components[ci].surrounding_comp_ids) {
                    if (!components[sid].faces.empty() && tag_set.count(components[sid].tag)) {
                        engulfing_comp_ids.insert(sid);
                        surrounding_tags.insert(components[sid].tag);
                    }
                }
            }
            // check surrounding tags is the same as tag_set
            if (surrounding_tags != tag_set) continue; // harmless hole, skip
            std::cout << "Engulfing_comp_ids: ";
            for (const auto& id : engulfing_comp_ids) {
                std::cout << id << " ";
            }
            std::cout << std::endl;
            std::cout << "Surrounding_tags: ";
            for (const auto& tag : surrounding_tags) {
                std::cout << tag << " ";
            }
            std::cout << std::endl;
            engulf_components(components, hole_cluster, engulfing_comp_ids);
        }
    }

    recompute_surface_info();
}

simplex::RawSimplexCollection ImageSimulationMeshTri::get_order1_edges_for_vertex(
    const size_t vid) const
{
    using namespace simplex;

    RawSimplexCollection sc;

    if (!m_vertex_attribute.at(vid).m_is_on_surface &&
        m_vertex_attribute.at(vid).on_bbox_faces.empty()) {
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

size_t ImageSimulationMeshTri::get_order_of_edge(const std::array<size_t, 2>& vids) const
{
    return (is_edge_on_surface(vids) || is_edge_on_bbox(vids)) ? 1 : 0;
}

size_t ImageSimulationMeshTri::get_order_of_vertex(const size_t vid) const
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

bool ImageSimulationMeshTri::substructure_link_condition(const Tuple& e_tuple) const
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

    RawSimplexCollection link_u_0;
    RawSimplexCollection link_u_1;
    RawSimplexCollection link_v_0;
    RawSimplexCollection link_v_1;
    RawSimplexCollection link_e_0;
    RawSimplexCollection link_e_1;

    constexpr size_t w_id = -1; // dummy vertex
    const Vertex w(w_id);

    const RawSimplexCollection u_surface_edges = get_order1_edges_for_vertex(u_id);
    const RawSimplexCollection v_surface_edges = get_order1_edges_for_vertex(v_id);

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

    const auto link_uv_0 = RawSimplexCollection::get_intersection(link_u_0, link_v_0);
    if (link_uv_0 != link_e_0) {
        return false;
    }
    const auto link_uv_1 = RawSimplexCollection::get_intersection(link_u_1, link_v_1);
    if (!link_uv_1.empty()) {
        return false;
    }

    return true;
}

} // namespace wmtk::components::image_simulation::tri