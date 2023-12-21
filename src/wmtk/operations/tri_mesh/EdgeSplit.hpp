#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeSplit;
}

template <>
struct OperationSettings<tri_mesh::EdgeSplit> : public OperationSettingsBase
{
    // constructor
    OperationSettings<tri_mesh::EdgeSplit>(TriMesh& m)
        : m_mesh(m)
    {}
    TriMesh& m_mesh;
    bool split_boundary_edges = true;
    void create_invariants();
};
namespace tri_mesh {
class EdgeSplit : public TriMeshOperation, protected TupleOperation
{
public:
    EdgeSplit(Mesh& m, const Simplex& t, const OperationSettings<EdgeSplit>& settings);

    std::string name() const override;


    std::vector<Tuple> triangle_onering() const;
    std::vector<Tuple> triangle_tworing() const;
    std::vector<Tuple> edge_onering() const;

    Tuple new_vertex() const;
    std::array<Tuple, 2> new_spine_edges() const;
    Tuple return_tuple() const;
    std::vector<Simplex> modified_primitives() const override;

    std::vector<Simplex> unmodified_primitives() const override;

    // std::vector<Tuple> new_triangles() const ;
    // std::vector<Tuple> new_edges() const ;

    // std::array<Tuple,2> spline_edges() const;
    // std::vector<Tuple> rib_edges() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

    using TriMeshOperation::hash_accessor;

protected:
    bool execute() override;

private:
    Tuple m_output_tuple;

    const OperationSettings<EdgeSplit>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
