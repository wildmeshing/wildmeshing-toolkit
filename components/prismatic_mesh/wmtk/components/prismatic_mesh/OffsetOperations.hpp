#pragma once

#include <wmtk/components/topological_offset/TopoOffsetTetMesh.h>

#include <array>
#include <cstddef>
#include <set>
#include <vector>

namespace wmtk::components::prismatic_mesh {

struct OffsetOperationStats
{
    size_t iterations = 0;
    size_t equal_source_attempts = 0;
    size_t equal_source_collapses = 0;
    size_t unlock_attempts = 0;
    size_t unlocks = 0;
    size_t unlock_rollbacks = 0;
    size_t remaining_equal_source_edges = 0;
    size_t remaining_tau_2_2 = 0;
};

/**
 * Prism-specific editing of a current topological-offset tetrahedral mesh.
 *
 * The paper's shell operations alter only tetrahedral connectivity. Hybrid cells
 * are recovered later, so this class deliberately does not add prism or pyramid
 * primitives to TetMesh.
 */
class PrismOffsetTetMesh final : public wmtk::components::topological_offset::TopoOffsetTetMesh
{
public:
    using Base = wmtk::components::topological_offset::TopoOffsetTetMesh;
    using Tuple = wmtk::TetMesh::Tuple;

    explicit PrismOffsetTetMesh(
        wmtk::components::topological_offset::Parameters& parameters,
        int num_threads = 0)
        : Base(parameters, num_threads)
    {}

    /** Apply collapse -> unlock passes until neither operation succeeds. */
    OffsetOperationStats simplify_offset(size_t max_iterations);

    /** One deterministic pass over equal-source offset-surface edges. */
    size_t collapse_equal_source_edges(OffsetOperationStats* stats = nullptr);

    /** One deterministic pass over paper configuration tau^2_2. */
    size_t unlock_tau_2_2(OffsetOperationStats* stats = nullptr);

    size_t count_equal_source_edges() const;
    size_t count_tau_2_2() const;
    std::set<int64_t> non_bijective_sources() const;

protected:
    bool collapse_edge_before(const Tuple& edge) override;
    bool collapse_edge_after(const Tuple& survivor) override;

private:
    enum class CollapseKind { None, EqualSource, Unlock };

    struct Tau22
    {
        std::array<size_t, 2> input;
        std::array<size_t, 2> offset;
        std::array<int64_t, 2> sources;
    };

    struct Snapshot
    {
        size_t vertex_count = 0;
        std::vector<std::array<size_t, 4>> tets;
        std::vector<wmtk::components::topological_offset::VertexAttributes> vertices;
        std::vector<wmtk::components::topological_offset::EdgeAttributes> edges;
        std::vector<wmtk::components::topological_offset::FaceAttributes> faces;
        std::vector<wmtk::components::topological_offset::TetAttributes> tet_attributes;
    };

    bool is_offset_surface_edge(const Tuple& edge) const;
    bool classify_tau_2_2(const Tuple& tet, Tau22& result) const;
    bool try_unlock(const Tau22& configuration, size_t diagonal);
    Snapshot snapshot() const;
    void restore(const Snapshot& state);

    CollapseKind m_collapse_kind = CollapseKind::None;
    size_t m_expected_removed = static_cast<size_t>(-1);
    size_t m_expected_survivor = static_cast<size_t>(-1);
    wmtk::components::topological_offset::VertexAttributes m_survivor_attributes;
};

} // namespace wmtk::components::prismatic_mesh
