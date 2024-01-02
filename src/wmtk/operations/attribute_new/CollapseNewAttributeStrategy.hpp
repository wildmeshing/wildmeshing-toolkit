#pragma once
#include <wmtk/multimesh/operations/CollapseReturnData.hpp>
#include <wmtk/multimesh/operations/extract_operation_tuples.hpp>
#include "CollapseNewAttributeTopoInfo.hpp"
#include "NewAttributeStrategy.hpp"

namespace wmtk::operations {
// default operation types, default specifies for rational/double we use averages , o/w copytuple
enum class CollapseBasicStrategy {
    Default,
    CopyTuple,
    CopyOther, // per-dimension "other" simplex option
    Mean,
    Throw,
    None
};

// This is necessary because subclass is templated
class BaseCollapseNewAttributeStrategy : public NewAttributeStrategy
{
public:
    using ReturnData = wmtk::multimesh::operations::CollapseReturnData;
    using OperationTupleData = wmtk::multimesh::operations::OperationTupleData;


    virtual void update(const ReturnData& ret_data, const OperationTupleData& tuples) = 0;
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

    CollapseNewAttributeStrategy(const wmtk::attribute::MeshAttributeHandle<T>& h);

    void update(const ReturnData& ret_data, const OperationTupleData& tuples) override;


    void set_strategy(CollapseFuncType&& f);
    void set_strategy(CollapseBasicStrategy t);

    Mesh& mesh() override;
    PrimitiveType primitive_type() const override;
    void update_handle_mesh(Mesh& m) override;
    bool matches_attribute(const attribute::MeshAttributeHandleVariant&) const override;

private:
    wmtk::attribute::MeshAttributeHandle<T> m_handle;
    CollapseFuncType m_collapse_op;
    std::unique_ptr<CollapseNewAttributeTopoInfo> m_topo_info;

    void assign_collapsed(
        PrimitiveType pt,
        const std::array<Tuple, 2>& input_simplices,
        const Tuple& final_simplex);

    static CollapseFuncType standard_collapse_strategy(CollapseBasicStrategy optype);
};
} // namespace wmtk::operations
