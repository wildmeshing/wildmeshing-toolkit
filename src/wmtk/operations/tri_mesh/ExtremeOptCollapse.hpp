#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include <wmtk/function/SYMDIR.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include <wmtk/operations/tri_mesh/TriMeshOperation.hpp>
#include "EdgeCollapse.hpp"


namespace wmtk::operations {
namespace tri_mesh {
class ExtremeOptCollapse;
}

template <>
struct OperationSettings<tri_mesh::ExtremeOptCollapse>
    : public OperationSettings<tri_mesh::EdgeCollapse>
{
    OperationSettings<tri_mesh::ExtremeOptCollapse>(TriMesh& m)
        : OperationSettings<tri_mesh::EdgeCollapse>(m)
    {}

    // handle to vertex position
    MeshAttributeHandle<double> position;
    // too long edges get ignored
    double max_squared_length = std::numeric_limits<double>::max();
    // in case of a collapse between an interior and a boundary vertex, the vertex is not moved to

    std::shared_ptr<TriMesh> uv_mesh_ptr;
    MeshAttributeHandle<double> uv_handle;
    bool optimize_E_max = true;
    void create_invariants();
};

namespace tri_mesh {
class ExtremeOptCollapse : public EdgeCollapse
{
public:
    ExtremeOptCollapse(
        Mesh& m,
        const Simplex& t,
        const OperationSettings<ExtremeOptCollapse>& settings);

    std::string name() const override;
    std::vector<double> priority() const override;
    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    std::tuple<
        std::vector<Tuple>,
        std::vector<Tuple>,
        std::vector<Eigen::VectorXd>,
        std::vector<Eigen::VectorXd>>
    cache_data_before_execute() const;
    bool check_branch_vertex_invariant(const Tuple& input_tuple_uv, bool& keep_v0) const;

    double get_energy_before() const;
    double get_energy_after() const;

    bool before() const override;
    bool execute() override;

private:
    Accessor<double> m_pos_accessor;
    Accessor<double> m_uv_accessor;

    const OperationSettings<ExtremeOptCollapse>& m_settings;
    std::shared_ptr<wmtk::function::SYMDIR> symdir_ptr;
};

} // namespace tri_mesh
} // namespace wmtk::operations
