#pragma once
#include <functional>
#include "SplitNewAttributeStrategy.hpp"

namespace wmtk::operations::tri_mesh {

template <typename T>
class BasicSplitNewAttributeStrategy : public SplitNewAttributeStrategy
{
public:
    enum class OpType {
        CopyTuple,
        CopyOther, // per-dimension "other" simplex option
        Mean,
        Custom
    };
    using VecType = VectorX<T>;
    using SplitRibFuncType = std::function<VecType(const VecType&, const VecType&)>;
    using SplitFuncType = std::function<std::array<VecType, 2>(const VecType&)>;
    BasicSplitNewAttributeStrategy(wmtk::attribute::MeshAttributeHandle<T>& h);

    void assign_split(
        PrimitiveType pt,
        const Tuple& input_simplex,
        const std::array<Tuple, 2>& split_simplices) override;


    void assign_split_ribs(
        PrimitiveType pt,
        const std::array<Tuple, 2>& input_ears,
        const Tuple& final_simplex) override;

    void set_split_rib_func(SplitRibFuncType&& f);
    void set_split_func(SplitFuncType&& f);

    void set_split_rib_type(OpType t);
    void set_split_type(OpType t);

private:
    wmtk::attribute::MeshAttributeHandle<T> m_handle;
    OpType m_split_optype = OpType::CopyTuple; // only copytuple makes sense

    OpType m_split_ribs_optype = std::is_same_v<T, double> ? OpType::Mean : OpType::CopyTuple;
    SplitRibFuncType m_split_rib_op;
    SplitFuncType m_split_op;
};


template <typename T>
BasicSplitNewAttributeStrategy(const wmtk::attribute::MeshAttributeHandle<T>&)
    -> BasicSplitNewAttributeStrategy<T>;
} // namespace wmtk::operations::tri_mesh
