#pragma once
#include <wmtk/multimesh/operations/CollapseReturnData.hpp>
#include <wmtk/multimesh/operations/extract_operation_tuples.hpp>
#include "CollapseNewAttributeTopoInfo.hpp"
#include "Enums.hpp"
#include "NewAttributeStrategy.hpp"

namespace wmtk::operations {

// This is necessary because subclass is templated
class BaseCollapseNewAttributeStrategy : public NewAttributeStrategy
{
public:
    using ReturnData = wmtk::multimesh::operations::CollapseReturnData;
    using OperationTupleData = wmtk::multimesh::operations::OperationTupleData;


    virtual void update(Mesh& m, const ReturnData& ret_data, const OperationTupleData& tuples)
        const = 0;
};

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

    void update(Mesh& m, const ReturnData& ret_data, const OperationTupleData& tuples)
        const override;


    void set_strategy(CollapseFuncType&& f);
    void set_strategy(CollapseBasicStrategy t);

    Mesh& mesh() override;
    using NewAttributeStrategy::mesh;
    PrimitiveType primitive_type() const override;
    void update_handle_mesh(Mesh& m) override;
    bool matches_attribute(const attribute::MeshAttributeHandle&) const override;

    std::vector<wmtk::attribute::MeshAttributeHandle> targets() const final override
    {
        return {m_handle};
    }

private:
    wmtk::attribute::MeshAttributeHandle m_handle;
    CollapseFuncType m_collapse_op;
    std::unique_ptr<CollapseNewAttributeTopoInfo> m_topo_info;

    void assign_collapsed(
        wmtk::attribute::Accessor<T>& acc,
        const std::array<Tuple, 2>& input_simplices,
        const Tuple& final_simplex) const;

    static CollapseFuncType standard_collapse_strategy(CollapseBasicStrategy optype);
};
} // namespace wmtk::operations
