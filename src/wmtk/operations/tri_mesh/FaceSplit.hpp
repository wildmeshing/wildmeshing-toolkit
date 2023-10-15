#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class FaceSplit;
}

template <>
struct OperationSettings<tri_mesh::FaceSplit>
{
    InvariantCollection invariants;
    void initialize_invariants(const TriMesh& m);
    // debug functionality to make sure operations are constructed properly
    bool are_invariants_initialized() const;
};

namespace tri_mesh {
class FaceSplit : public TriMeshOperation, private TupleOperation
{
public:
    FaceSplit(Mesh& m, const Tuple& t, const OperationSettings<FaceSplit>& settings);

    std::string name() const override;

    // the return tuple is the new vertex, arrow to the original vertex
    // this operation never set the position of the new vertex.
    //     / | \
    //    /  |  \
    //   /  _*_  \
    //  / _< f \_ \
    //  |/_______\|
    //   \       /
    //    \     /
    //     \   /
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Face; }

protected:
    bool execute() override;
    bool before() const override;

private:
    Tuple m_output_tuple;
    const OperationSettings<FaceSplit>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations