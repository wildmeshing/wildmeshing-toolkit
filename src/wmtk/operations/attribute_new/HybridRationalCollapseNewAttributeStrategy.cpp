#include "HybridRationalCollapseNewAttributeStrategy.hpp"
#include <wmtk/attribute/utils/HybridRationalAccessor.hpp>
#include <wmtk/utils/primitive_range.hpp>

#include <wmtk/operations/edge_mesh/CollapseNewAttributeTopoInfo.hpp>
#include <wmtk/operations/tet_mesh/CollapseNewAttributeTopoInfo.hpp>
#include <wmtk/operations/tri_mesh/CollapseNewAttributeTopoInfo.hpp>

namespace wmtk::operations {

typename HybridRationalCollapseNewAttributeStrategy::CollapseFuncType
HybridRationalCollapseNewAttributeStrategy::standard_collapse_strategy(CollapseBasicStrategy optype)
{
    switch (optype) {
    default: [[fallthrough]];
    case CollapseBasicStrategy::Default:
        return standard_collapse_strategy(CollapseBasicStrategy::CopyTuple);
    case CollapseBasicStrategy::CopyTuple:
        return [](const ConstMapValueType& a,
                  const ConstMapValueType& b,
                  const std::bitset<2>& bs) -> ResultValueType {
            if (!bs[1] && bs[0]) {
                return b;
            } else {
                return a;
            }
        };
    case CollapseBasicStrategy::CopyOther:
        return [](const ConstMapValueType& a,
                  const ConstMapValueType& b,
                  const std::bitset<2>& bs) -> ResultValueType {
            if (!bs[0] && bs[1]) {
                return a;
            } else {
                return b;
            }
        };
    case CollapseBasicStrategy::Mean:
        assert(false);
        // return [](const ConstMapValueType&, const ConstMapValueType&, const std::bitset<2>&) ->
        // ResultValueType {
        //     throw std::runtime_error("Collapse with mean on ");
        // };
    case CollapseBasicStrategy::Throw:
        return [](const ConstMapValueType&,
                  const ConstMapValueType&,
                  const std::bitset<2>&) -> ResultValueType {
            throw std::runtime_error("Collapse should have a new attribute");
        };
    case CollapseBasicStrategy::None: return {};
    }
    return {};
}


HybridRationalCollapseNewAttributeStrategy::HybridRationalCollapseNewAttributeStrategy(
    const wmtk::attribute::MeshAttributeHandle& h)
    : m_handle(h)
    , m_collapse_op(nullptr)
{
    set_strategy(CollapseBasicStrategy::Throw);

    auto& mesh = m_handle.mesh();

    if (mesh.top_simplex_type() == PrimitiveType::Edge) {
        m_topo_info =
            std::make_unique<edge_mesh::CollapseNewAttributeTopoInfo>(static_cast<EdgeMesh&>(mesh));
    } else if (mesh.top_simplex_type() == PrimitiveType::Triangle) {
        m_topo_info =
            std::make_unique<tri_mesh::CollapseNewAttributeTopoInfo>(static_cast<TriMesh&>(mesh));
    } else if (mesh.top_simplex_type() == PrimitiveType::Tetrahedron) {
        m_topo_info =
            std::make_unique<tet_mesh::CollapseNewAttributeTopoInfo>(static_cast<TetMesh&>(mesh));
    } else {
        throw std::runtime_error("Invalid mesh");
    }
}

void HybridRationalCollapseNewAttributeStrategy::update(
    const ReturnData& data,
    const OperationTupleData& op_datas)
{
    if (op_datas.find(&mesh()) == op_datas.end()) return;
    const std::vector<std::array<Tuple, 2>>& tuple_pairs = op_datas.at(&mesh());

    for (const auto& tuple_pair : tuple_pairs) {
        const Tuple& input_tuple = tuple_pair[0];
        const Tuple& output_tuple = tuple_pair[1];

        const auto& return_data_variant =
            data.get_variant(mesh(), wmtk::simplex::Simplex::edge(input_tuple));

        for (const PrimitiveType pt : wmtk::utils::primitive_below(mesh().top_simplex_type())) {
            auto merged_simps = m_topo_info->merged_simplices(return_data_variant, input_tuple, pt);
            auto new_simps = m_topo_info->new_simplices(return_data_variant, output_tuple, pt);


            assert(merged_simps.size() == new_simps.size());

            for (size_t s = 0; s < merged_simps.size(); ++s) {
                assign_collapsed(pt, merged_simps[s], new_simps[s]);
            }
        }
    }
}


void HybridRationalCollapseNewAttributeStrategy::assign_collapsed(
    PrimitiveType pt,
    const std::array<Tuple, 2>& input_simplices,
    const Tuple& final_simplex)
{
    if (!bool(m_collapse_op)) {
        return;
    }
    if (pt != primitive_type()) {
        return;
    }
    wmtk::attribute::utils::HybridRationalAccessor acc(
        m_handle.mesh(),
        m_handle.as_from_held_type<HybridHeldType>());
    auto old_values = m_handle.mesh().parent_scope([&]() {
        return std::make_tuple(
            acc.const_value(input_simplices[0]),
            acc.const_value(input_simplices[1]));
    });

    const auto old_pred = this->evaluate_predicate(pt, input_simplices);

    auto [a, b] = old_values;
    auto new_value = acc.value(final_simplex);


    new_value = m_collapse_op(a, b, old_pred);
}


void HybridRationalCollapseNewAttributeStrategy::set_strategy(CollapseFuncType&& f)
{
    m_collapse_op = std::move(f);
}
void HybridRationalCollapseNewAttributeStrategy::set_strategy(CollapseBasicStrategy optype)
{
    set_strategy(standard_collapse_strategy(optype));
}


Mesh& HybridRationalCollapseNewAttributeStrategy::mesh()
{
    return m_handle.mesh();
}
PrimitiveType HybridRationalCollapseNewAttributeStrategy::primitive_type() const
{
    return m_handle.primitive_type();
}
void HybridRationalCollapseNewAttributeStrategy::update_handle_mesh(Mesh& m)
{
    m_handle =
        wmtk::attribute::MeshAttributeHandle(m, m_handle.as_from_held_type<HybridHeldType>());
}
bool HybridRationalCollapseNewAttributeStrategy::matches_attribute(
    const attribute::MeshAttributeHandle& handle) const
{
    return handle == m_handle;
}


} // namespace wmtk::operations
