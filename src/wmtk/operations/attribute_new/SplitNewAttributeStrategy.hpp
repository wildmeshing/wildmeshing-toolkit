#pragma once
#include <wmtk/multimesh/operations/SplitReturnData.hpp>
#include <wmtk/multimesh/operations/extract_operation_tuples.hpp>
#include "NewAttributeStrategy.hpp"
#include "SplitNewAttributeTopoInfo.hpp"


namespace wmtk::operations {

// default operation types
enum class SplitBasicStrategy { Default, Copy, Half, Throw, None };
//rib and collapse have hte same prototypes / default funs available
enum class SplitRibBasicStrategy {
    Default,
    CopyTuple,
    CopyOther, // per-dimension "other" simplex option
    Mean,
    Throw,
    None
};

// This is necessary because subclass is templated
class BaseSplitNewAttributeStrategy : public NewAttributeStrategy
{
public:
    using ReturnData = wmtk::multimesh::operations::SplitReturnData;
    using OperationTupleData = wmtk::multimesh::operations::OperationTupleData;


    virtual void update(const ReturnData& ret_data, const OperationTupleData& op_data) = 0;
};

template <typename T>
class SplitNewAttributeStrategy : public BaseSplitNewAttributeStrategy
{
public:
    using VecType = VectorX<T>;
    using ReturnData = BaseSplitNewAttributeStrategy::ReturnData;
    using OperationTupleData = BaseSplitNewAttributeStrategy::OperationTupleData;
    using ReturnVariant = ReturnData::ReturnVariant;

    // given two ear $k$-simplices, define a value for the single new $k$-simplex between them
    using SplitRibFuncType =
        std::function<VecType(const VecType&, const VecType&, const std::bitset<2>&)>;

    // given a k-simplex that was split in two, provide new values for thw two simplices created
    // from it first one is the one that "shares" a vertex with the op's "input tuple
    using SplitFuncType =
        std::function<std::array<VecType, 2>(const VecType&, const std::bitset<2>&)>;


    SplitNewAttributeStrategy(const wmtk::attribute::MeshAttributeHandle& h);

    void update(const ReturnData& ret_data, const OperationTupleData& op_data) override;

    void set_rib_strategy(SplitRibFuncType&& f);
    void set_strategy(SplitFuncType&& f);

    void set_rib_strategy(SplitRibBasicStrategy t);
    void set_strategy(SplitBasicStrategy t);


    Mesh& mesh() override;
    PrimitiveType primitive_type() const override;
    void update_handle_mesh(Mesh& m) override;
    bool matches_attribute(const attribute::MeshAttributeHandle&) const override;

private:
    wmtk::attribute::MeshAttributeHandle m_handle;
    SplitRibFuncType m_split_rib_op;
    SplitFuncType m_split_op;
    std::unique_ptr<SplitNewAttributeTopoInfo> m_topo_info;

    void assign_split(
        PrimitiveType pt,
        const Tuple& input_simplex,
        const std::array<Tuple, 2>& split_simplices);

    void assign_split_ribs(
        PrimitiveType pt,
        const std::array<Tuple, 2>& input_ears,
        const Tuple& final_simplex);

    static SplitFuncType standard_split_strategy(SplitBasicStrategy optype);
    static SplitRibFuncType standard_split_rib_strategy(SplitRibBasicStrategy optype);
};


} // namespace wmtk::operations
