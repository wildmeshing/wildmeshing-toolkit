#pragma once
#include <wmtk/multimesh/operations/SplitReturnData.hpp>
#include <wmtk/multimesh/operations/extract_operation_tuples.hpp>
#include "Enums.hpp"
#include "NewAttributeStrategy.hpp"
#include "SplitNewAttributeTopoInfo.hpp"


namespace wmtk::operations {


// This is necessary because subclass is templated
class BaseSplitNewAttributeStrategy : public NewAttributeStrategy
{
public:
    using ReturnData = wmtk::multimesh::operations::SplitReturnData;
    using OperationTupleData = wmtk::multimesh::operations::OperationTupleData;


    virtual void update(Mesh& m, const ReturnData& ret_data, const OperationTupleData& op_data)
        const = 0;
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

    void update(Mesh& m, const ReturnData& ret_data, const OperationTupleData& op_data)
        const final override;

    void set_rib_strategy(SplitRibFuncType&& f);
    void set_strategy(SplitFuncType&& f);

    void set_rib_strategy(SplitRibBasicStrategy t);
    void set_strategy(SplitBasicStrategy t);


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
    SplitRibFuncType m_split_rib_op;
    SplitFuncType m_split_op;
    std::unique_ptr<SplitNewAttributeTopoInfo> m_topo_info;

    void assign_split(
        wmtk::attribute::Accessor<T>& accessor,
        const Tuple& input_simplex,
        const std::array<Tuple, 2>& split_simplices) const;

    void assign_split_ribs(
        wmtk::attribute::Accessor<T>& accessor,
        const std::array<Tuple, 2>& input_ears,
        const Tuple& final_simplex) const;

    static SplitFuncType standard_split_strategy(
        SplitBasicStrategy optype,
        const std::string_view& = {});
    static SplitRibFuncType standard_split_rib_strategy(
        SplitRibBasicStrategy optype,
        const std::string_view& = {});
};


} // namespace wmtk::operations
