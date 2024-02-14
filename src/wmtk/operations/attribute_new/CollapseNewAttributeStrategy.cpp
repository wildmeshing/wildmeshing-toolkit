#include "CollapseNewAttributeStrategy.hpp"
#include <wmtk/utils/primitive_range.hpp>

#include <wmtk/operations/edge_mesh/CollapseNewAttributeTopoInfo.hpp>
#include <wmtk/operations/tet_mesh/CollapseNewAttributeTopoInfo.hpp>
#include <wmtk/operations/tri_mesh/CollapseNewAttributeTopoInfo.hpp>

namespace wmtk::operations {

template <typename T>
typename CollapseNewAttributeStrategy<T>::CollapseFuncType
CollapseNewAttributeStrategy<T>::standard_collapse_strategy(CollapseBasicStrategy optype)
{
    switch (optype) {
    default: [[fallthrough]];
    case CollapseBasicStrategy::Default:
        if constexpr (std::is_same_v<T, double> || std::is_same_v<T, Rational>) {
            return standard_collapse_strategy(CollapseBasicStrategy::Mean);
        } else {
            return standard_collapse_strategy(CollapseBasicStrategy::CopyTuple);
        }
    case CollapseBasicStrategy::CopyTuple:
        return [](const VecType& a, const VecType& b, const std::bitset<2>& bs) -> VecType {
            if (!bs[1] && bs[0]) {
                return b;
            } else {
                return a;
            }
        };
    case CollapseBasicStrategy::CopyOther:
        return [](const VecType& a, const VecType& b, const std::bitset<2>& bs) -> VecType {
            if (!bs[0] && bs[1]) {
                return a;
            } else {
                return b;
            }
        };
    case CollapseBasicStrategy::Mean:
        return [](const VecType& a, const VecType& b, const std::bitset<2>& bs) -> VecType {
            if (bs[0] == bs[1]) {
                return (a + b) / T(2);
            } else if (bs[0]) {
                return a;

            } else {
                return b;
            }
        };
    case CollapseBasicStrategy::Throw:
        return [](const VecType&, const VecType&, const std::bitset<2>&) -> VecType {
            throw std::runtime_error("Collapse should have a new attribute");
        };
    case CollapseBasicStrategy::None: return {};
    }
    return {};
}


template <typename T>
CollapseNewAttributeStrategy<T>::CollapseNewAttributeStrategy(
    const wmtk::attribute::MeshAttributeHandle& h)
    : m_handle(h)
    , m_collapse_op(nullptr)
{
    assert(h.holds<T>());
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

template <typename T>
void CollapseNewAttributeStrategy<T>::update(
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


template <typename T>
void CollapseNewAttributeStrategy<T>::assign_collapsed(
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
    auto acc = m_handle.mesh().create_accessor(m_handle.as<T>());
    auto old_values = m_handle.mesh().parent_scope([&]() {
        return std::make_tuple(
            acc.const_vector_attribute(input_simplices[0]),
            acc.const_vector_attribute(input_simplices[1]));
    });

    const auto old_pred = this->evaluate_predicate(pt, input_simplices);

    VecType a, b;
    std::tie(a, b) = old_values;
    auto new_value = acc.vector_attribute(final_simplex);


    new_value = m_collapse_op(a, b, old_pred);
}


template <typename T>
void CollapseNewAttributeStrategy<T>::set_strategy(CollapseFuncType&& f)
{
    m_collapse_op = std::move(f);
}
template <typename T>
void CollapseNewAttributeStrategy<T>::set_strategy(CollapseBasicStrategy optype)
{
    set_strategy(standard_collapse_strategy(optype));
}


template <typename T>
Mesh& CollapseNewAttributeStrategy<T>::mesh()
{
    return m_handle.mesh();
}
template <typename T>
PrimitiveType CollapseNewAttributeStrategy<T>::primitive_type() const
{
    return m_handle.primitive_type();
}
template <typename T>
void CollapseNewAttributeStrategy<T>::update_handle_mesh(Mesh& m)
{
    m_handle = wmtk::attribute::MeshAttributeHandle(m, m_handle.as<T>());
}
template <typename T>
bool CollapseNewAttributeStrategy<T>::matches_attribute(
    const attribute::MeshAttributeHandle& handle) const
{
    return handle == m_handle;
}

template class CollapseNewAttributeStrategy<char>;
template class CollapseNewAttributeStrategy<int64_t>;
template class CollapseNewAttributeStrategy<double>;
template class CollapseNewAttributeStrategy<Rational>;

} // namespace wmtk::operations
