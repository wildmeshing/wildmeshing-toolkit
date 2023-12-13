#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "EdgeCollapse.hpp"
#include "EdgeSplit.hpp"
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class FaceSplit;
} // namespace tri_mesh


template <>
struct OperationSettings<tri_mesh::FaceSplit> : public OperationSettingsBase
{
    OperationSettings<tri_mesh::FaceSplit>(TriMesh& m)
        : m_mesh(m)
        , split_settings(m)
        , collapse_settings(m)
    {}

    OperationSettings<tri_mesh::EdgeSplit> split_settings;
    OperationSettings<tri_mesh::EdgeCollapse> collapse_settings;

    TriMesh& m_mesh;

    void create_invariants();
};

namespace tri_mesh {
/**
 * The return tuple is the new vertex, pointing to the original vertex.
 * This operation does not set vertex positions.
 *     / | \
 *    /  |  \
 *   /  _*_  \
 *  / _< f \_ \
 *  |/_______\|
 *   \       /
 *    \     /
 *     \   /
 **/
class FaceSplit : public TriMeshOperation, private TupleOperation
{
public:
    FaceSplit(Mesh& m, const Simplex& t, const OperationSettings<FaceSplit>& settings);

    std::string name() const override;
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Face; }
    std::vector<Simplex> modified_primitives() const override;

protected:
    bool execute() override;
    bool before() const override;

private:
    Tuple m_output_tuple;
    const OperationSettings<FaceSplit>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
