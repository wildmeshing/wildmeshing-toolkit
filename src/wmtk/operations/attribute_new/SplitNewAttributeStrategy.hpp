#pragma once
#include <wmtk/multimesh/operations/SplitReturnData.hpp>
#include <wmtk/multimesh/operations/extract_operation_tuples.hpp>
#include "NewAttributeStrategy.hpp"


namespace wmtk::operations {

template <typename T>
class SplitNewAttributeStrategy : public NewAttributeStrategy
{
public:
    // default operation types
    enum class SplitBasicStrategy { Default, Copy, Half, Throw, None };
    //rib and collapse have hte same prototypes / default funs available
    using SplitRibBasicStrategy = CollapseBasicStrategy;

    using VecType = VectorX<T>;

    // given two ear $k$-simplices, define a value for the single new $k$-simplex between them
    using SplitRibFuncType =
        std::function<VecType(const VecType&, const VecType&, const std::bitset<2>&)>;

    // given a k-simplex that was split in two, provide new values for thw two simplices created
    // from it first one is the one that "shares" a vertex with the op's "input tuple
    using SplitFuncType =
        std::function<std::array<VecType, 2>(const VecType&, const std::bitset<2>&)>;


    SplitNewAttributeStrategy(const wmtk::attribute::MeshAttributeHandle<T>& h);

    void update(const ReturnData& ret_data, const OperationTupleData& op_data);

    void update_handle_mesh(Mesh& m);

    void set_split_rib_strategy(SplitRibFuncType&& f);
    void set_split_strategy(SplitFuncType&& f);

    void set_split_rib_strategy(SplitRibBasicStrategy t);
    void set_split_strategy(SplitBasicStrategy t);


    Mesh& mesh() override;
    PrimitiveType primitive_type() const override;
    void update_handle_mesh(Mesh& m) override;
    bool matches_attribute(const attribute::MeshAttributeHandleVariant&) const override;

private:
    wmtk::attribute::MeshAttributeHandle<T> m_handle;
    SplitRibFuncType m_split_rib_op;
    SplitFuncType m_split_op;
    SplitNewAttributeTopoInfo m_topo_info;

    void assign_split(
        PrimitiveType pt,
        const Tuple& input_simplex,
        const std::array<Tuple, 2>& split_simplices);

    void assign_split_ribs(
        PrimitiveType pt,
        const std::array<Tuple, 2>& input_ears,
        const Tuple& final_simplex);
};


} // namespace wmtk::operations
