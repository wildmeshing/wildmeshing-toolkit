#pragma once

#include <wmtk/TriMesh.h>
#include <wmtk/envelope/Envelope.hpp>

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <vector>

namespace wmtk {

/**
 * @brief A tube around the open boundary of a triangle mesh, for operations allowed to move it.
 *
 * The alternative to freezing the boundary. Freezing keeps the outline exactly, and with it
 * everything enclosed by boundary loops packed too tightly to leave a free vertex between them.
 * The tube lets the outline coarsen and move instead, as long as every boundary edge that
 * descends from the input's boundary stays within eps of the input's boundary edges. Holes and
 * slits narrower than the tube can then close.
 *
 * "Descends from" is a per-vertex lineage flag, `input_boundary`, kept in the mesh's own vertex
 * attributes so that it is rolled back and consolidated with them. The operations maintain it:
 *
 *   - init() sets it on every vertex of a boundary edge of the input;
 *   - a collapse ORs it into the survivor;
 *   - a split of a boundary edge gives it to the new vertex when either endpoint has it;
 *   - swaps and smoothing leave it alone.
 *
 * A boundary edge derived from the input's outline then always touches a flagged vertex, and
 * boundary_edges_inside() judges exactly those. Boundary that an operation opens elsewhere --
 * a collapse with the link condition off tearing a dense closed patch, say -- is not judged, as
 * it was not judged under a frozen boundary either: such tears are transient while a patch
 * simplifies, and refusing them leaves it stuck (Thingi10K 55928; see
 * ShortestEdgeCollapse::create_mesh).
 *
 * The vertex attributes are duck-typed: anything indexed by vertex id whose elements have an
 * Eigen::Vector3d `pos` and a bool `input_boundary` -- in practice a component's
 * AttributeCollection<VertexAttributes>.
 */
class BoundaryEnvelope
{
public:
    /**
     * @brief Build the tube around the boundary edges of @p m and flag their vertices.
     *
     * Call right after TriMesh::init, before any operation.
     *
     * @param eps       tube radius
     * @param use_exact whether queries use the exact predicate. An edge envelope builds its
     *                  exact structure only when this is set here, so set it to whatever the
     *                  surface envelope uses; set_use_exact() can switch to the sampled
     *                  predicate later, but not back.
     * @return whether there is a boundary. A closed mesh builds nothing and flags nothing, so
     *         there is nothing to judge either, and initialized() stays false.
     */
    template <typename VertexAttributes>
    bool init(const TriMesh& m, VertexAttributes& vertex_attrs, double eps, bool use_exact)
    {
        std::vector<Eigen::Vector2i> E;
        for (const TriMesh::Tuple& e : m.get_edges()) {
            if (!m.is_boundary_edge(e)) continue;
            const size_t a = e.vid(m), b = e.switch_vertex(m).vid(m);
            E.emplace_back(int(a), int(b));
            vertex_attrs[a].input_boundary = true;
            vertex_attrs[b].input_boundary = true;
        }
        if (E.empty()) {
            m_initialized = false;
            return false;
        }
        std::vector<Eigen::Vector3d> V(m.vert_capacity());
        for (size_t i = 0; i < V.size(); ++i) {
            V[i] = vertex_attrs[i].pos;
        }
        build(V, E, eps, use_exact);
        return true;
    }

    /// Whether init() found a boundary to build the tube around.
    bool initialized() const { return m_initialized; }

    /**
     * Answer queries with the exact predicate or the sampled one, to follow a surface envelope
     * whose flag the caller flips after init() (tetwild does, around its simplification). The
     * exact structure exists only if use_exact was on at init().
     */
    void set_use_exact(bool use_exact) { m_envelope.use_exact = use_exact; }

    /**
     * @brief Every boundary edge of @p tris that touches an input_boundary vertex lies in the
     * tube.
     *
     * Meant for TriMesh::invariants(), with the triangles an operation produced: they include
     * every edge whose position or boundary status the operation changed. Edges touching no
     * flagged vertex are skipped before the boundary test, so a mesh far from its boundary pays
     * one flag lookup per edge.
     */
    template <typename VertexAttributes>
    bool boundary_edges_inside(
        const TriMesh& m,
        const VertexAttributes& vertex_attrs,
        const std::vector<TriMesh::Tuple>& tris) const
    {
        if (!m_initialized) return true;
        for (const TriMesh::Tuple& t : tris) {
            for (int j = 0; j < 3; ++j) {
                const TriMesh::Tuple e = m.tuple_from_edge(t.fid(m), j);
                const size_t a = e.vid(m), b = e.switch_vertex(m).vid(m);
                if (!vertex_attrs[a].input_boundary && !vertex_attrs[b].input_boundary) continue;
                if (!m.is_boundary_edge(e)) continue;
                const std::array<Eigen::Vector3d, 2> seg{
                    {vertex_attrs[a].pos, vertex_attrs[b].pos}};
                if (m_envelope.is_outside(seg)) return false;
            }
        }
        return true;
    }

    /**
     * @brief @p v is an input_boundary vertex that is still on the boundary.
     *
     * For placement: an operation that moves such a vertex off the boundary pulls the outline
     * into the surface, which the tube then refuses, so the operation is better off keeping it
     * where it is. Only flagged vertices pay for the ring walk.
     *
     * The ring walk reads the connectivity of @p v's neighbours, so in a parallel pass ask it
     * only where those are locked -- of an endpoint of the edge an operation is about to change,
     * say, not of the edges it hands back for re-queueing.
     */
    template <typename VertexAttributes>
    static bool on_input_boundary(
        const TriMesh& m,
        const VertexAttributes& vertex_attrs,
        const TriMesh::Tuple& v)
    {
        return vertex_attrs[v.vid(m)].input_boundary && m.is_boundary_vertex(v);
    }

private:
    void build(
        const std::vector<Eigen::Vector3d>& V,
        const std::vector<Eigen::Vector2i>& E,
        double eps,
        bool use_exact);

    SampleEnvelope m_envelope;
    bool m_initialized = false;
};

} // namespace wmtk
