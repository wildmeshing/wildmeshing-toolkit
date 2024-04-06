#pragma once
#include "BaseSplitNewAttributeStrategy.hpp"


namespace wmtk::operations {

class HybridRationalSplitNewAttributeStrategy : public BaseSplitNewAttributeStrategy
{
public:
    using HeldType = wmtk::attribute::MeshAttributeHandle::HeldType;
    constexpr static HeldType HybridHeldType = HeldType::HybridRational;
    using HybridAttributeHandleType =
        wmtk::attribute::MeshAttributeHandle::held_handle_type<HybridHeldType>;
    using MapValueType = typename HybridAttributeHandleType::MapValueType;
    using ConstMapValueType = typename HybridAttributeHandleType::ConstMapValueType;
    using ResultValueType = typename HybridAttributeHandleType::ResultValueType;
    using ReturnData = BaseSplitNewAttributeStrategy::ReturnData;
    using OperationTupleData = BaseSplitNewAttributeStrategy::OperationTupleData;
    using ReturnVariant = ReturnData::ReturnVariant;

    // given two ear $k$-simplices, define a value for the single new $k$-simplex between them
    using SplitRibFuncType = std::function<
        ResultValueType(const ConstMapValueType&, const ConstMapValueType&, const std::bitset<2>&)>;

    // given a k-simplex that was split in two, provide new values for thw two simplices created
    // from it first one is the one that "shares" a vertex with the op's "input tuple
    using SplitFuncType = std::function<
        std::array<ResultValueType, 2>(const ConstMapValueType&, const std::bitset<2>&)>;


    HybridRationalSplitNewAttributeStrategy(const wmtk::attribute::MeshAttributeHandle& h);

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
