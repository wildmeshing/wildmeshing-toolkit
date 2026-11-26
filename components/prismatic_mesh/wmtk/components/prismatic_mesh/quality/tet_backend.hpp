#pragma once
// Fixed-domain adapter. Scheduling, topology operations and acceptance criteria
// are inherited from TetOptimizerMesh; the sizing callback is TetWildMesh's.
#include <wmtk/TetOptimizerMesh.h>
#include <chrono>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <set>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SizingField.hpp>

namespace wmtk::components::prismatic_mesh::quality {
using Json = nlohmann::json;
using Tet = std::array<size_t, 4>;
class StockFixedBackground : public wmtk::TetOptimizerMesh
{
    struct Metadata
    {
        int64_t original = -1;
        bool pinned = false;
    };
    using Face = std::array<int64_t, 3>;
    wmtk::AttributeCollection<double> qualities;
    wmtk::AttributeCollection<Metadata> metadata;
    std::set<Face> source_boundary;
    std::map<int64_t, wmtk::Vector3d> source_pins;
    std::chrono::steady_clock::time_point started = std::chrono::steady_clock::now();
    std::string progress_path;
    size_t splits = 0;
    size_t split_budget = std::numeric_limits<size_t>::max();
    std::function<bool()> should_stop;

public:
    bool local_sizing_splits_only = false;
    Json samples = Json::array(), refinements = Json::array();
    void set_budget(size_t limit, std::function<bool()> stop)
    {
        split_budget = limit;
        should_stop = std::move(stop);
    }
    StockFixedBackground(
        wmtk::OptimizerParameters& params,
        const std::vector<wmtk::Vector3d>& points,
        const std::vector<Tet>& cells,
        const std::vector<uint8_t>& fixed,
        std::string progress = {},
        const std::vector<double>& sizing = {})
        : TetOptimizerMesh(params, nullptr)
        , progress_path(std::move(progress))
    {
        NUM_THREADS = 0;
        m_vertex_attr_group.add(&metadata);
        p_tet_attrs = &qualities;
        set_preallocation_factor(3);
        init_with_isolated_vertices(points.size(), cells);
        for (size_t v = 0; v < points.size(); ++v) {
            auto& a = m_vertex_attribute[v];
            a.m_posf = points[v];
            a.m_is_rounded = true;
            a.set_pos_to_posf();
            a.m_is_on_surface = false;
            a.m_order = 0;
            a.m_sizing_scalar = sizing.empty() ? 1 : sizing.at(v);
            a.on_bbox_faces.clear();
            a.partition_id = 0;
            metadata[v].original = int64_t(v);
            metadata[v].pinned = fixed.at(v);
        }
        for (const auto& f : get_faces())
            if (f.is_boundary_face(*this))
                for (const auto& v : get_face_vertices(f)) metadata[v.vid(*this)].pinned = true;
        for (const auto& v : get_vertices())
            if (metadata[v.vid(*this)].pinned)
                source_pins[metadata[v.vid(*this)].original] =
                    m_vertex_attribute[v.vid(*this)].m_posf;
        source_boundary = boundary();
        for (const auto& t : get_tets()) {
            if (is_inverted(t)) throw std::runtime_error("Nonpositive input background tet.");
            qualities[t.tid(*this)] = get_quality(t);
        }
    }
    double cell_quality(size_t tid) const override { return qualities[tid]; }
    void set_cell_quality(size_t tid, double q) override { qualities[tid] = q; }
    std::shared_ptr<wmtk::Envelope> smoothing_energy_envelope(size_t) const override
    {
        return nullptr;
    }
    bool allow_surface_swap() const override { return false; }
    bool check_surface_topology() const override { return false; }
    void write_optimization_debug_output(const std::string&) override {}
    bool split_edge_before(const Tuple& e) override
    {
        if (local_sizing_splits_only) {
            const auto a = e.vid(*this), b = e.switch_vertex(*this).vid(*this);
            if (.5 * (m_vertex_attribute[a].m_sizing_scalar +
                      m_vertex_attribute[b].m_sizing_scalar) >=
                1 - 1e-12)
                return false;
        }
        return splits < split_budget && !(should_stop && should_stop()) &&
               !e.is_boundary_edge(*this) && TetOptimizerMesh::split_edge_before(e);
    }
    void split_after_vertex(size_t v, bool) override
    {
        metadata[v] = Metadata{};
        if (++splits % 10000 == 0)
            logger().debug("Tet remeshing splits: {}, seconds {}", splits, seconds());
    }
    bool collapse_before_vertex(size_t removed, size_t, double) override
    {
        const auto& a = metadata;
        return !(should_stop && should_stop()) && !a[removed].pinned;
    }
    bool smooth_before(const Tuple& v) override
    {
        const auto& a = metadata;
        return !(should_stop && should_stop()) && !a[v.vid(*this)].pinned &&
               TetOptimizerMesh::smooth_before(v);
    }

    // Same sizing-refinement implementation and shared helpers as TetWildMesh.cpp.
    // The adapter adds fixed-boundary pins, a per-stage split budget and cancellation.
    size_t refine_sizing_around_worst(double max_energy) override
    {
        const int n_rings = std::max(0, m_params.stuck_refine_rings);
        const double filter = std::min(std::max(max_energy / 100, m_params.stop_energy), 100.);
        const auto worst = wmtk::utils::select_worst_cells(
            tet_capacity(),
            [this](size_t t) { return tuple_from_tet(t).is_valid(*this); },
            [this](size_t t) { return std::cbrt(cell_quality(t)); },
            filter,
            m_params.stuck_refine_num_worst);
        if (worst.empty()) return 0;
        m_force_split_edges.clear();
        if (m_params.stuck_refine_force_split)
            for (const auto& [q, t] : worst)
                m_force_split_edges.insert(
                    wmtk::utils::longest_edge(
                        oriented_tet_vids(t),
                        [this](size_t v) -> const wmtk::Vector3d& {
                            return m_vertex_attribute[v].m_posf;
                        }));
        std::vector<size_t> seeds;
        seeds.reserve(4 * worst.size());
        for (const auto& [q, t] : worst)
            for (size_t v : oriented_tet_vids(t)) seeds.push_back(v);
        const auto region = wmtk::utils::grow_vertex_region(seeds, n_rings, [this](size_t v) {
            return get_one_ring_vids_for_vertex_adj(v);
        });
        const auto refined = wmtk::utils::apply_sizing_refinement(
            region,
            m_params.stuck_refine_factor,
            m_params.stuck_refine_min_scalar,
            [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; });
        gradation_smooth_sizing(m_params.stuck_refine_gradation, refined);
        refinements.push_back(
            {{"iteration", m_iterations_used},
             {"worst_cells", worst.size()},
             {"refined_vertices", refined.size()},
             {"forced_edges", m_force_split_edges.size()},
             {"maximum_AMIPS", max_energy},
             {"seconds", seconds()}});
        logger().debug("Tet sizing refinement: {}", refinements.back().dump());
        write_progress();
        return refined.size();
    }
    std::tuple<double, double> optimization_quality_stats() override
    {
        const auto result = TetOptimizerMesh::optimization_quality_stats();
        samples.push_back(measure());
        logger().debug("Tet quality: {}", samples.back().dump());
        write_progress();
        return result;
    }
    double seconds() const
    {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    }
    Json measure() const
    {
        size_t bad = 0, bad1000 = 0, bad10000 = 0, unrounded = 0;
        double mx = 0;
        long double sum = 0;
        const auto cells = get_tets();
        const auto vertices = get_vertices();
        for (const auto& t : cells) {
            const double q = std::cbrt(cell_quality(t.tid(*this)));
            mx = std::max(mx, q);
            sum += q;
            bad += q > 100;
            bad1000 += q > 1000;
            bad10000 += q > 10000;
        }
        for (const auto& v : vertices) unrounded += !m_vertex_attribute[v.vid(*this)].m_is_rounded;
        return {
            {"iteration", m_iterations_used},
            {"pass", m_debug_pass_name},
            {"cells", cells.size()},
            {"points", vertices.size()},
            {"maximum_AMIPS", mx},
            {"average_AMIPS", double(sum / cells.size())},
            {"over_100", bad},
            {"over_1000", bad1000},
            {"over_10000", bad10000},
            {"unrounded_vertices", unrounded},
            {"seconds", seconds()}};
    }
    Json report() const
    {
        Json counts;
        for (int k = 0; k < int(OpKind::COUNT); ++k)
            counts[op_kind_name(OpKind(k))] = op_event_count(OpKind(k), OpEvent::committed);
        return {
            {"backend", "TetOptimizerMesh::local_operations, stock operation acceptance"},
            {"iterations_used", m_iterations_used},
            {"final", measure()},
            {"accepted_topology_operations", counts},
            {"samples", samples},
            {"refinements", refinements},
            {"split_generation_budget", split_budget},
            {"splits_committed", splits},
            {"boundary_vertices_and_faces_fixed_by_policy", true}};
    }
    void write_progress() const
    {
        if (!progress_path.empty()) std::ofstream(progress_path) << report().dump(2) << '\n';
    }
    std::set<Face> boundary() const
    {
        std::set<Face> faces;
        for (const auto& f : get_faces())
            if (f.is_boundary_face(*this)) {
                Face ids;
                const auto v = get_face_vertices(f);
                for (int j = 0; j < 3; ++j) ids[j] = metadata[v[j].vid(*this)].original;
                std::sort(ids.begin(), ids.end());
                faces.insert(ids);
            }
        return faces;
    }
    void validate_fixed_boundary() const
    {
        if (boundary() != source_boundary)
            throw std::runtime_error("Background boundary triangulation changed.");
        std::set<int64_t> found;
        for (const auto& v : get_vertices()) {
            const auto& a = metadata[v.vid(*this)];
            if (!a.pinned) continue;
            const auto it = source_pins.find(a.original);
            if (it == source_pins.end() || it->second != m_vertex_attribute[v.vid(*this)].m_posf)
                throw std::runtime_error("Pinned background point moved.");
            found.insert(a.original);
        }
        if (found.size() != source_pins.size())
            throw std::runtime_error("Pinned background point removed.");
    }
    void export_data(
        std::vector<wmtk::Vector3d>& points,
        std::vector<int64_t>& original,
        std::vector<Tet>& cells,
        std::vector<double>* sizing = nullptr,
        std::vector<std::array<size_t, 2>>* forced = nullptr) const
    {
        std::vector<size_t> numbering(vert_capacity(), size_t(-1));
        for (const auto& v : get_vertices()) {
            const auto id = v.vid(*this);
            numbering[id] = points.size();
            points.push_back(m_vertex_attribute[id].m_posf);
            original.push_back(metadata[id].original);
            if (sizing) sizing->push_back(m_vertex_attribute[id].m_sizing_scalar);
        }
        for (const auto& t : get_tets()) {
            auto c = oriented_tet_vids(t);
            for (auto& v : c) v = numbering[v];
            cells.push_back(c);
        }
        if (forced)
            for (const auto& e : m_force_split_edges)
                forced->push_back({numbering.at(e.vertices()[0]), numbering.at(e.vertices()[1])});
    }
    void import_forced_edges(const std::vector<std::array<size_t, 2>>& edges)
    {
        for (const auto& e : edges) m_force_split_edges.insert(wmtk::simplex::Edge(e[0], e[1]));
    }
};

} // namespace wmtk::components::prismatic_mesh::quality
