#pragma once

#include <igl/Timer.h>
#include <wmtk/components/topological_offset/Parameters.h>
#include <wmtk/components/topological_offset/TopoOffsetTriMesh.h>
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


class ManExtractTriMesh : public topological_offset::TopoOffsetTriMesh
{
public:
    int m_vtu_counter = 0;
    int m_surfvtu_counter = 0;

    Parameters& m_man_params;

    ManExtractTriMesh(
        Parameters& _m_man_params,
        topological_offset::Parameters& _m_top_params,
        int _num_threads = 0)
        : topological_offset::TopoOffsetTriMesh(_m_top_params, _num_threads)
        , m_man_params(_m_man_params)
    {}

    ~ManExtractTriMesh() {}

public:
    /**
     * @brief label nonmanifold simplices (edges and verts). Corresponding attributes have label set
     * to 1
     */
    size_t label_non_manifold(); // return # of non manifold edges, verts

    /**
     * @brief check if a vertex is manifold. A vertex is manifold if all 'inside' and all 'outside'
     * tets adjacent to the vertex form single face-reachable connected components, respectively.
     * For vertices on the boundary of the mesh, external space is considered an 'outside' polyhedra
     * that is face connected to all tets on the mesh boundary.
     */
    bool vertex_is_manifold(const Tuple& v) const;

    /**
     * @brief dfs helper for vertex manifold check
     */
    void vertex_dfs_helper(std::set<size_t>& visited_fids, const Tuple& f) const;

    /**
     * @brief extract edge mesh separating 'inside' faces from 'outside' faces. Resulting curve is
     * guaranteed to be manifold.
     */
    void extract_curve_mesh(MatrixXd& V, MatrixXi& F) const;

    /**
     * @brief write curve .obj (boundary of in_tag)
     */
    void write_curve(const std::string& path);

private: // helpers
    bool is_curve_vertex(const Tuple& v) const;
    bool is_curve_edge(const Tuple& e) const;
    bool is_interior_face(const Tuple& f) const;
    bool is_interior_face(size_t f_id) const;
};

} // namespace wmtk::components::manifold_extraction
