#pragma once
#include "BaseCollapseNewAttributeStrategy.hpp"

namespace wmtk::operations {

template <typename T>
class CollapseNewAttributeStrategy : public BaseCollapseNewAttributeStrategy
{
public:
    using ReturnData = BaseCollapseNewAttributeStrategy::ReturnData;
    using OperationTupleData = BaseCollapseNewAttributeStrategy::OperationTupleData;
    using ReturnVariant = ReturnData::ReturnVariant;

    using VecType = VectorX<T>;

    // given two k-simplices that were merged into one, provide new values for that new simplex.
    // first argument is the one that "shares" a vertex with the op's "input tuple
    using CollapseFuncType =
        std::function<VecType(const VecType&, const VecType&, const std::bitset<2>&)>;

    CollapseNewAttributeStrategy(const wmtk::attribute::MeshAttributeHandle& h);

    void update(const ReturnData& ret_data, const OperationTupleData& tuples) override;


    void set_strategy(CollapseFuncType&& f);
    void set_strategy(CollapseBasicStrategy t);

    Mesh& mesh() override;
    PrimitiveType primitive_type() const override;
    void update_handle_mesh(Mesh& m) override;
    bool matches_attribute(const attribute::MeshAttributeHandle&) const override;

private:
    wmtk::attribute::MeshAttributeHandle m_handle;
    CollapseFuncType m_collapse_op;
    std::unique_ptr<CollapseNewAttributeTopoInfo> m_topo_info;

    void assign_collapsed(
        PrimitiveType pt,
        const std::array<Tuple, 2>& input_simplices,
        const Tuple& final_simplex);

    static CollapseFuncType standard_collapse_strategy(CollapseBasicStrategy optype);
};
} // namespace wmtk::operations
