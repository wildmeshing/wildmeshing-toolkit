
#pragma once
#include <optional>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeCollapse;
}

template <>
struct OperationSettings<tri_mesh::EdgeCollapse> : public OperationSettingsBase
{
    OperationSettings<tri_mesh::EdgeCollapse>(TriMesh& m)
        : m_mesh(m)
    {}

    TriMesh& m_mesh;

    // are collapses between boundary and interior vertices allowed
    bool collapse_boundary_vertex_to_interior = true;
    // are collapses on boundary edges allowed
    bool collapse_boundary_edges = true;
        // are collapses preserving topology
    bool preserve_topology = false;
    // are collapses preserving geometry
    bool preserve_geometry = false;

    void create_invariants();
};

namespace tri_mesh {
class EdgeCollapse : public TriMeshOperation, protected TupleOperation
{
public:
    // constructor for default factory pattern construction
    EdgeCollapse(Mesh& m, const Simplex& t, const OperationSettings<EdgeCollapse>& settings);
    EdgeCollapse(TriMesh& m, const Simplex& t, const OperationSettings<EdgeCollapse>& settings);

    std::string name() const override;


    std::vector<Tuple> modified_triangles() const;
    std::vector<Tuple> modified_primitives(PrimitiveType) const override;

    // return next-->opposite tuple if it exists, otherwise return previous-->opposite
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

    using TriMeshOperation::hash_accessor;

protected:
    bool execute() override;

protected:
    Tuple m_output_tuple;
    // const OperationSettings<EdgeCollapse>& m_settings; // TODO unused variable
};

} // namespace tri_mesh
} // namespace wmtk::operations
