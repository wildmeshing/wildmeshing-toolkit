#include "SplitNewAttributeStrategy.hpp"
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/primitive_range.hpp>

#include <wmtk/operations/edge_mesh/SplitNewAttributeTopoInfo.hpp>
#include <wmtk/operations/tet_mesh/SplitNewAttributeTopoInfo.hpp>
#include <wmtk/operations/tri_mesh/SplitNewAttributeTopoInfo.hpp>


namespace wmtk::operations {

template <typename T>
bool SplitNewAttributeStrategy<T>::invalid_state() const
{
    return m_will_throw || m_will_throw_rib;
}
template <typename T>
std::string SplitNewAttributeStrategy<T>::name() const
{
    return fmt::format("SplitNewAttributeStrategy[{}]", m_handle.name());
}

template <typename T>
typename SplitNewAttributeStrategy<T>::SplitFuncType SplitNewAttributeStrategy<
    T>::standard_split_strategy(SplitBasicStrategy optype, const std::string_view& name)
{
    using VT = SplitNewAttributeStrategy::VecType;

    switch (optype) {
    default: [[fallthrough]];
    case SplitBasicStrategy::Default: [[fallthrough]];
    case SplitBasicStrategy::Copy:
        return [](const VT& a, const std::bitset<2>&) -> std::array<VT, 2> {
            return std::array<VT, 2>{{a, a}};
        };
    case SplitBasicStrategy::Half:
        return [](const VT& a, const std::bitset<2>&) -> std::array<VT, 2> {
            return std::array<VT, 2>{{a / T(2), a / T(2)}};
        };
    case SplitBasicStrategy::Throw: {
        std::string mn(name);
        return [mn](const VT&, const std::bitset<2>&) -> std::array<VT, 2> {
            log_and_throw_error("Split should have a new attribute for [{}]", mn);
        };
    }
    case SplitBasicStrategy::None: return {};
    }
    return {};
}

template <>
typename SplitNewAttributeStrategy<Rational>::SplitFuncType SplitNewAttributeStrategy<
    Rational>::standard_split_strategy(SplitBasicStrategy optype, const std::string_view& name)
{
    using VT = SplitNewAttributeStrategy::VecType;

    switch (optype) {
    default: [[fallthrough]];
    case SplitBasicStrategy::Default: [[fallthrough]];
    case SplitBasicStrategy::Copy:
        return [](const VT& a, const std::bitset<2>&) -> std::array<VT, 2> {
            return std::array<VT, 2>{{a, a}};
        };
    case SplitBasicStrategy::Half:
        return [](const VT& a, const std::bitset<2>&) -> std::array<VT, 2> {
            return std::array<VT, 2>{{a / Rational(2, true), a / Rational(2, true)}};
        };
    case SplitBasicStrategy::Throw: {
        std::string mn(name);
        return [mn](const VT&, const std::bitset<2>&) -> std::array<VT, 2> {
            log_and_throw_error("Split should have a new attribute for [{}]", mn);
        };
    }
    case SplitBasicStrategy::None: return {};
    }
    return {};
}


template <typename T>
typename SplitNewAttributeStrategy<T>::SplitRibFuncType SplitNewAttributeStrategy<
    T>::standard_split_rib_strategy(SplitRibBasicStrategy optype, const std::string_view& name)
{
    using VT = SplitNewAttributeStrategy::VecType;

    switch (optype) {
    default: [[fallthrough]];
    case SplitRibBasicStrategy::Default:
        if constexpr (std::is_same_v<T, double> || std::is_same_v<T, Rational>) {
            return standard_split_rib_strategy(SplitRibBasicStrategy::Mean);
        } else {
            return standard_split_rib_strategy(SplitRibBasicStrategy::CopyTuple);
        }
    case SplitRibBasicStrategy::CopyTuple:
        return [](const VT& a, const VT& b, const std::bitset<2>& bs) -> VT {
            // if both are boundary then return a (failed link anyway but oh well)
            // if a is boundary but b is interior get b though
            if (!bs[1] && bs[0]) {
                return b;
            } else {
                return a;
            }
        };
    case SplitRibBasicStrategy::CopyOther:
        return [](const VT& a, const VT& b, const std::bitset<2>& bs) -> VT {
            if (!bs[0] && bs[1]) {
                return a;
            } else {
                return b;
            }
        };
    case SplitRibBasicStrategy::Mean:
        return [](const VT& a, const VT& b, const std::bitset<2>& bs) -> VT {
            if (bs[0] == bs[1]) {
                return (a + b) / T(2);
            } else if (bs[0]) {
                return a;

            } else {
                return b;
            }
        };
    case SplitRibBasicStrategy::Throw: {
        std::string mn(name);
        return [mn](const VT&, const VT&, const std::bitset<2>&) -> VT {
            log_and_throw_error("Split should have a new attribute [{}]", mn);
        };
    }
    case SplitRibBasicStrategy::None: return {};
    }
    return {};
}
template <>
typename SplitNewAttributeStrategy<Rational>::SplitRibFuncType
SplitNewAttributeStrategy<Rational>::standard_split_rib_strategy(
    SplitRibBasicStrategy optype,
    const std::string_view& name)
{
    using VT = SplitNewAttributeStrategy::VecType;

    switch (optype) {
    default: [[fallthrough]];
    case SplitRibBasicStrategy::Default:
        return standard_split_rib_strategy(SplitRibBasicStrategy::Mean);
    case SplitRibBasicStrategy::CopyTuple:
        return [](const VT& a, const VT& b, const std::bitset<2>& bs) -> VT {
            // if both are boundary then return a (failed link anyway but oh well)
            // if a is boundary but b is interior get b though
            if (!bs[1] && bs[0]) {
                return b;
            } else {
                return a;
            }
        };
    case SplitRibBasicStrategy::CopyOther:
        return [](const VT& a, const VT& b, const std::bitset<2>& bs) -> VT {
            if (!bs[0] && bs[1]) {
                return a;
            } else {
                return b;
            }
        };
    case SplitRibBasicStrategy::Mean:
        return [](const VT& a, const VT& b, const std::bitset<2>& bs) -> VT {
            if (bs[0] == bs[1]) {
                return (a + b) / Rational(2, true);
            } else if (bs[0]) {
                return a;

            } else {
                return b;
            }
        };
    case SplitRibBasicStrategy::Throw: {
        std::string mn(name);
        return [mn](const VT&, const VT&, const std::bitset<2>&) -> VT {
            log_and_throw_error("Split should have a new attribute [{}]", mn);
        };
    }
    case SplitRibBasicStrategy::None: return {};
    }
    return {};
}


template <typename T>
SplitNewAttributeStrategy<T>::SplitNewAttributeStrategy(
    const wmtk::attribute::MeshAttributeHandle& h)
    : m_handle(h)
    , m_split_rib_op(nullptr)
    , m_split_op(nullptr)
{
    assert(h.holds<T>());
    set_rib_strategy(SplitRibBasicStrategy::Throw);
    set_strategy(SplitBasicStrategy::Throw);

    auto& mesh = m_handle.mesh();
    assert(
        !mesh.is_free() || m_handle.primitive_type() ==
                               PrimitiveType::Vertex); // attribute new is not valid on free meshes

    if (mesh.top_simplex_type() == PrimitiveType::Edge) {
        m_topo_info =
            std::make_unique<edge_mesh::SplitNewAttributeTopoInfo>(static_cast<EdgeMesh&>(mesh));
    } else if (mesh.top_simplex_type() == PrimitiveType::Triangle) {
        m_topo_info =
            std::make_unique<tri_mesh::SplitNewAttributeTopoInfo>(static_cast<TriMesh&>(mesh));
    } else if (mesh.top_simplex_type() == PrimitiveType::Tetrahedron) {
        m_topo_info =
            std::make_unique<tet_mesh::SplitNewAttributeTopoInfo>(static_cast<TetMesh&>(mesh));
    } else {
        log_and_throw_error("Invalid mesh");
    }
}

template <typename T>
void SplitNewAttributeStrategy<T>::update(
    Mesh& m,
    const ReturnData& data,
    const OperationInOutData& op_datas) const
{
    if (!bool(m_split_rib_op) && !bool(m_split_op)) {
        return;
    }

    if (op_datas.find(&mesh()) == op_datas.end()) return;
    assert(&m == &mesh());
    const std::vector<std::tuple<simplex::NavigatableSimplex, Tuple>>& tuple_pairs =
        op_datas.at(&mesh());

    const PrimitiveType pt = primitive_type();
    auto acc = m.create_accessor(m_handle.as<T>());
    for (const auto& tuple_pair : tuple_pairs) {
        const simplex::NavigatableSimplex& input_simplex = std::get<0>(tuple_pair);
        const Tuple& output_tuple = std::get<1>(tuple_pair);


        const auto& return_data_variant = data.get_variant(mesh(), input_simplex);

        // for (const PrimitiveType pt : wmtk::utils::primitive_below(mesh().top_simplex_type()))
        {
            // copy attributes opposing ears
            auto old_simps =
                m_topo_info->input_ear_simplices(return_data_variant, input_simplex.tuple(), pt);
            auto new_simps =
                m_topo_info->output_rib_simplices(return_data_variant, output_tuple, pt);


            assert(old_simps.size() == new_simps.size());

            for (size_t s = 0; s < old_simps.size(); ++s) {
                assign_split_ribs(acc, old_simps[s], new_simps[s]);
            }
        }
        {
            auto old_simps =
                m_topo_info->input_split_simplices(return_data_variant, input_simplex.tuple(), pt);
            auto new_simps =
                m_topo_info->output_split_simplices(return_data_variant, output_tuple, pt);


            assert(old_simps.size() == new_simps.size());

            for (size_t s = 0; s < old_simps.size(); ++s) {
                assign_split(acc, old_simps[s], new_simps[s]);
            }
        }
        if (mesh().is_free()) {
            assert(m_handle.primitive_type() == PrimitiveType::Vertex);

            auto pairs = m_topo_info->output_duplicated_free_simplices(return_data_variant, pt);
            for (const auto& [first, second] : pairs) {
                acc.vector_attribute(second) = acc.const_vector_attribute(first);
            }
        }
    }
}


template <typename T>
void SplitNewAttributeStrategy<T>::assign_split_ribs(
    wmtk::attribute::Accessor<T>& acc,
    const std::array<Tuple, 2>& input_ears,
    const Tuple& final_simplex) const
{
    if (!bool(m_split_rib_op)) {
        return;
    }
    assert(acc.primitive_type() == primitive_type());

    auto old_values = m_handle.mesh().parent_scope([&]() {
        return std::make_tuple(
            acc.const_vector_attribute(input_ears[0]),
            acc.const_vector_attribute(input_ears[1]));
    });

    VecType a, b;
    std::tie(a, b) = old_values;
    auto new_value = acc.vector_attribute(final_simplex);

    const auto old_pred = this->evaluate_predicate(acc.primitive_type(), input_ears);

    new_value = m_split_rib_op(a, b, old_pred);
}

template <typename T>
void SplitNewAttributeStrategy<T>::assign_split(
    wmtk::attribute::Accessor<T>& acc,
    const Tuple& input_simplex,
    const std::array<Tuple, 2>& split_simplices) const
{
    if (!bool(m_split_op)) {
        return;
    }
    assert(acc.primitive_type() == primitive_type());
    const VecType old_value =
        m_handle.mesh().parent_scope([&]() { return acc.const_vector_attribute(input_simplex); });

    std::bitset<2> pred = this->evaluate_predicate(acc.primitive_type(), split_simplices);

    auto arr = m_split_op(old_value, pred);
    for (size_t j = 0; j < 2; ++j) {
        acc.vector_attribute(split_simplices[j]) = arr[j];
    }
}


template <typename T>
void SplitNewAttributeStrategy<T>::set_rib_strategy(SplitRibBasicStrategy t)
{
    set_rib_strategy(standard_split_rib_strategy(t, m_handle.name()));
    m_will_throw_rib = t == SplitRibBasicStrategy::Throw;
}
template <typename T>
void SplitNewAttributeStrategy<T>::set_strategy(SplitBasicStrategy t)
{
    set_strategy(standard_split_strategy(t, m_handle.name()));
    m_will_throw = t == SplitBasicStrategy::Throw;
}

template <typename T>
void SplitNewAttributeStrategy<T>::set_rib_strategy(SplitRibFuncType&& f)
{
    m_split_rib_op = std::move(f);
    m_will_throw_rib = false;
}
template <typename T>
void SplitNewAttributeStrategy<T>::set_strategy(SplitFuncType&& f)
{
    m_split_op = std::move(f);
    m_will_throw = false;
}

template <typename T>
Mesh& SplitNewAttributeStrategy<T>::mesh()
{
    return m_handle.mesh();
}

template <typename T>
PrimitiveType SplitNewAttributeStrategy<T>::primitive_type() const
{
    return m_handle.primitive_type();
}

template <typename T>
void SplitNewAttributeStrategy<T>::update_handle_mesh(Mesh& m)
{
    m_handle = wmtk::attribute::MeshAttributeHandle(m, m_handle.handle());
}

template <typename T>
bool SplitNewAttributeStrategy<T>::matches_attribute(
    const attribute::MeshAttributeHandle& handle) const
{
    return handle == m_handle;
}


template class SplitNewAttributeStrategy<char>;
template class SplitNewAttributeStrategy<int64_t>;
template class SplitNewAttributeStrategy<double>;
template class SplitNewAttributeStrategy<Rational>;

} // namespace wmtk::operations
