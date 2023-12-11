#pragma once
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class ExtremeOptSplit;
}

template <>
struct OperationSettings<tri_mesh::ExtremeOptSplit>
{
    OperationSettings<tri_mesh::EdgeSplit> split_settings;
    // handle to vertex position
    MeshAttributeHandle<double> position;
    std::shared_ptr<TriMesh> uv_mesh_ptr;
    MeshAttributeHandle<double> uv_handle;
    // too short edges get ignored
    double min_squared_length = -1;

    void initialize_invariants(const TriMesh& m);

    // debug functionality to make sure operations are constructed properly
    bool are_invariants_initialized() const;
};

namespace tri_mesh {
class ExtremeOptSplit : public EdgeSplit
{
public:
    ExtremeOptSplit(Mesh& m, const Tuple& t, const OperationSettings<ExtremeOptSplit>& settings);

    std::string name() const override;

    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool before() const override;
    bool execute() override;

private:
    Tuple m_output_tuple;
    TriMesh& m_mesh;
    Accessor<double> m_pos_accessor;
    Accessor<double> m_uv_accessor;
    const OperationSettings<ExtremeOptSplit>& m_settings;

    Eigen::VectorXd coord0;
    Eigen::VectorXd coord1;

    std::vector<Eigen::VectorXd> coord0s_uv;
    std::vector<Eigen::VectorXd> coord1s_uv;
};

} // namespace tri_mesh
} // namespace wmtk::operations
