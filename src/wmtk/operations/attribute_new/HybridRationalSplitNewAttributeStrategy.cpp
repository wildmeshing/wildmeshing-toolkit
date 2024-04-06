#include "HybridRationalSplitNewAttributeStrategy.hpp"
#include <wmtk/attribute/utils/HybridRationalAccessor.hpp>

#include <wmtk/utils/primitive_range.hpp>
#include "SplitNewAttributeStrategy.hpp"

#include <wmtk/operations/edge_mesh/SplitNewAttributeTopoInfo.hpp>
#include <wmtk/operations/tet_mesh/SplitNewAttributeTopoInfo.hpp>
#include <wmtk/operations/tri_mesh/SplitNewAttributeTopoInfo.hpp>


namespace wmtk::operations {


typename HybridRationalSplitNewAttributeStrategy::SplitFuncType
HybridRationalSplitNewAttributeStrategy::standard_split_strategy(SplitBasicStrategy optype)
{
    switch (optype) {
    default: [[fallthrough]];
    case SplitBasicStrategy::Default: [[fallthrough]];
    case SplitBasicStrategy::Copy:
        return [](const ConstMapValueType& a,
                  const std::bitset<2>&) -> std::array<ResultValueType, 2> {
            return std::array<ResultValueType, 2>{{a, a}};
        };
    case SplitBasicStrategy::Half:
        return [](const ConstMapValueType& a,
                  const std::bitset<2>&) -> std::array<ResultValueType, 2> {
            return {};
            //return std::array<ResultValueType, 2>{{a / T(2), a / T(2)}};
        };
    case SplitBasicStrategy::Throw:
        return [](const ConstMapValueType& a,
                  const std::bitset<2>&) -> std::array<ResultValueType, 2> {
            throw std::runtime_error("Split should have a new attribute");
        };
    case SplitBasicStrategy::None: return {};
    }
    return {};
}

typename HybridRationalSplitNewAttributeStrategy::SplitRibFuncType
HybridRationalSplitNewAttributeStrategy::standard_split_rib_strategy(SplitRibBasicStrategy optype)
{
    switch (optype) {
    default: [[fallthrough]];
    case SplitRibBasicStrategy::Default:
        return standard_split_rib_strategy(SplitRibBasicStrategy::Mean);
        // for split we pick average as rational if anything is rational
    case SplitRibBasicStrategy::CopyTuple:
        return [](const ConstMapValueType& a,
                  const ConstMapValueType& b,
                  const std::bitset<2>& bs) -> ResultValueType {
            // if both are boundary then return a (failed link anyway but oh well)
            // if a is boundary but b is interior get b though
            if (!bs[1] && bs[0]) {
                return b;
            } else {
                return a;
            }
        };
    case SplitRibBasicStrategy::CopyOther:
        return [](const ConstMapValueType& a,
                  const ConstMapValueType& b,
                  const std::bitset<2>& bs) -> ResultValueType {
            if (!bs[0] && bs[1]) {
                return a;
            } else {
                return b;
            }
        };
    case SplitRibBasicStrategy::Mean:
        return [](const ConstMapValueType& a,
                  const ConstMapValueType& b,
                  const std::bitset<2>& bs) -> ResultValueType {
            if (bs[0] == bs[1]) {
                auto [ca, ra, da] = a;
                auto [cb, rb, db] = b;
                const bool a_rational = (ca.array() == 1).any();
                const bool b_rational = (cb.array() == 1).any();
                ResultValueType retvalue;
                auto [c, r, d] = retvalue;
                // make sure every entry is sized properly
                c.resize(ca.rows());
                r.resize(ca.rows());
                d.resize(ca.rows());

                if (a_rational || b_rational) {
                    c.setZero();
                    if (a_rational && b_rational) {
                        // both rational
                        r = (ra + rb) / 2.0;
                    } else if (a_rational) {
                        // a rational, b not

                        auto rat_b =
                            db.unaryExpr([](const double& v) { return wmtk::Rational(v); });
                        r = (ra + rat_b) / 2.0;
                    } else if (b_rational) {
                        // b rational, a not
                        auto rat_a =
                            da.unaryExpr([](const double& v) { return wmtk::Rational(v); });
                        r = (rb + rat_a) / 2.0;
                    } else {
                    }
                } else {
                    c.setOnes();
                }

                return retvalue;
            } else if (bs[0]) {
                return a;

            } else {
                return b;
            }
        };
    case SplitRibBasicStrategy::Throw:
        return [](const ConstMapValueType& a,
                  const ConstMapValueType& b,
                  const std::bitset<2>& bs) -> ResultValueType {
            throw std::runtime_error("Split should have a new attribute");
        };
    case SplitRibBasicStrategy::None: return {};
    }
    return {};
}


HybridRationalSplitNewAttributeStrategy::HybridRationalSplitNewAttributeStrategy(
    const wmtk::attribute::MeshAttributeHandle& h)
    : m_handle(h)
    , m_split_rib_op(nullptr)
    , m_split_op(nullptr)
{
    set_rib_strategy(SplitRibBasicStrategy::Throw);
    set_strategy(SplitBasicStrategy::Throw);

    auto& mesh = m_handle.mesh();

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
        throw std::runtime_error("Invalid mesh");
    }
}

void HybridRationalSplitNewAttributeStrategy::update(
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
            {
                auto old_simps =
                    m_topo_info->input_ear_simplices(return_data_variant, input_tuple, pt);
                auto new_simps =
                    m_topo_info->output_rib_simplices(return_data_variant, output_tuple, pt);


                assert(old_simps.size() == new_simps.size());

                for (size_t s = 0; s < old_simps.size(); ++s) {
                    assign_split_ribs(pt, old_simps[s], new_simps[s]);
                }
            }
            {
                auto old_simps =
                    m_topo_info->input_split_simplices(return_data_variant, input_tuple, pt);
                auto new_simps =
                    m_topo_info->output_split_simplices(return_data_variant, output_tuple, pt);


                assert(old_simps.size() == new_simps.size());

                for (size_t s = 0; s < old_simps.size(); ++s) {
                    assign_split(pt, old_simps[s], new_simps[s]);
                }
            }
        }
    }
}


void HybridRationalSplitNewAttributeStrategy::assign_split_ribs(
    PrimitiveType pt,
    const std::array<Tuple, 2>& input_ears,
    const Tuple& final_simplex)
{
    if (!bool(m_split_rib_op)) {
        return;
    }
    if (pt != primitive_type()) {
        return;
    }

    wmtk::attribute::utils::HybridRationalAccessor acc(
        m_handle.mesh(),
        m_handle.as_from_held_type<HybridHeldType>());
    auto old_values = m_handle.mesh().parent_scope([&]() {
        return std::make_tuple(acc.const_value(input_ears[0]), acc.const_value(input_ears[1]));
    });

    auto [a, b] = old_values;
    auto new_value = acc.value(final_simplex);

    const auto old_pred = this->evaluate_predicate(pt, input_ears);

    new_value = m_split_rib_op(a, b, old_pred);
}

void HybridRationalSplitNewAttributeStrategy::assign_split(
    PrimitiveType pt,
    const Tuple& input_simplex,
    const std::array<Tuple, 2>& split_simplices)
{
    if (!bool(m_split_op)) {
        return;
    }
    if (pt != primitive_type()) {
        return;
    }
    wmtk::attribute::utils::HybridRationalAccessor acc(
        m_handle.mesh(),
        m_handle.as_from_held_type<HybridHeldType>());
    const ConstMapValueType old_value =
        m_handle.mesh().parent_scope([&]() { return acc.const_value(input_simplex); });

    std::bitset<2> pred = this->evaluate_predicate(pt, split_simplices);

    auto arr = m_split_op(old_value, pred);
    for (size_t j = 0; j < 2; ++j) {
        acc.value(split_simplices[j]) = arr[j];
    }
}


void HybridRationalSplitNewAttributeStrategy::set_rib_strategy(SplitRibBasicStrategy t)
{
    set_rib_strategy(standard_split_rib_strategy(t));
}
void HybridRationalSplitNewAttributeStrategy::set_strategy(SplitBasicStrategy t)
{
    set_strategy(standard_split_strategy(t));
}

void HybridRationalSplitNewAttributeStrategy::set_rib_strategy(SplitRibFuncType&& f)
{
    m_split_rib_op = std::move(f);
}
void HybridRationalSplitNewAttributeStrategy::set_strategy(SplitFuncType&& f)
{
    m_split_op = std::move(f);
}

Mesh& HybridRationalSplitNewAttributeStrategy::mesh()
{
    return m_handle.mesh();
}

PrimitiveType HybridRationalSplitNewAttributeStrategy::primitive_type() const
{
    return m_handle.primitive_type();
}

void HybridRationalSplitNewAttributeStrategy::update_handle_mesh(Mesh& m)
{
    m_handle = wmtk::attribute::MeshAttributeHandle(m, m_handle.handle());
}

bool HybridRationalSplitNewAttributeStrategy::matches_attribute(
    const attribute::MeshAttributeHandle& handle) const
{
    return handle == m_handle;
}


} // namespace wmtk::operations
