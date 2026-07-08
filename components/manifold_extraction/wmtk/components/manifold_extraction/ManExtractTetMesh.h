#pragma once

#include <igl/Timer.h>
#include <wmtk/components/topological_offset/Parameters.h>
#include <wmtk/components/topological_offset/TopoOffsetTetMesh.h>
#include <wmtk/utils/Morton.h>
#include <wmtk/utils/PartitionMesh.h>
#include <bitset>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include "Parameters.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/concurrent_map.h>
#include <tbb/parallel_sort.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

using namespace wmtk::components;


namespace wmtk::components::manifold_extraction {


class ManExtractTetMesh : public topological_offset::TopoOffsetTetMesh
{
public:
    int m_vtu_counter = 0;
    int m_surfvtu_counter = 0;
    bool m_boundary_input_vert_found = false;

    Parameters& m_man_params;

    ManExtractTetMesh(
        Parameters& _m_man_params,
        topological_offset::Parameters& _m_top_params,
        int _num_threads = 0)
        : topological_offset::TopoOffsetTetMesh(_m_top_params, _num_threads)
        , m_man_params(_m_man_params)
    {}

    ~ManExtractTetMesh() {}

public:
    /**
     * @brief label nonmanifold simplices (edges and verts). Corresponding attributes have label set
     * to 1
     */
    std::pair<size_t, size_t> label_non_manifold(); // return # of non manifold edges, verts

    /**
     * @brief check if an edge is manifold, ie if all 'inside' tets adjacent to the edge form
     * single face-reachable connected component
     */
    bool edge_is_manifold(const Tuple& t) const;

    /**
     * @brief dfs helper for edge manifold check
     */
    void edge_dfs_helper(std::set<size_t>& visited_tids, const Tuple& t) const;

    /**
     * @brief check if a vertex is manifold. A vertex is manifold if all 'inside' and all 'outside'
     * tets adjacent to the vertex form single face-reachable connected components, respectively.
     * For vertices on the boundary of the mesh, external space is considered an 'outside' polyhedra
     * that is face connected to all tets on the mesh boundary.
     */
    bool vertex_is_manifold(const Tuple& t) const;

    /**
     * @brief dfs helper for vertex manifold check
     */
    void vertex_dfs_helper(
        std::set<size_t>& visited_tids,
        const Tuple& t,
        const bool include,
        const std::vector<simplex::Face>& b_out_faces) const;

    /**
     * @brief check if a vertex is on the boundary of the mesh
     */
    bool is_boundary_vertex(size_t vid) const;

    /**
     * @brief collect faces on the boundary of the mesh that belong to an 'outside' tet.
     */
    std::vector<simplex::Face> get_boundary_faces_for_out_tets(size_t vid) const;

    /**
     * @brief extract tri mesh separating 'inside' tets from 'outside' tets. Resulting mesh is
     * guaranteed to be manifold.
     */
    void extract_surface_mesh(MatrixXd& V, MatrixXi& F);

    /**
     * @brief write surface (boundary of in_tag)
     */
    void write_surface(const std::string& path);

private: // helpers
    bool is_surface_vertex(const Tuple& v) const;
    bool is_surface_edge(const Tuple& e) const;
    bool is_surface_face(const Tuple& f) const;
    bool is_interior_tet(const Tuple& t) const;
    bool is_interior_tet(const size_t& t_id) const;
};

} // namespace wmtk::components::manifold_extraction
