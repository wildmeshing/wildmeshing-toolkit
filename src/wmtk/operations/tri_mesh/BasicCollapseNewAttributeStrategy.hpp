#pragma once
#include "CollapseNewAttributeStrategy.hpp"

namespace wmtk::operations::tri_mesh {

template <typename T>
class BasicCollapseNewAttributeStrategy : public CollapseNewAttributeStrategy
{
public:
    using VecType = VectorX<T>;
    using CollapseFuncType = std::function<VecType(const VecType&, const VecType&)>;
    BasicCollapseNewAttributeStrategy(wmtk::attribute::MeshAttributeHandle<T>& h);

    void assign_collapsed(
        PrimitiveType pt,
        const std::array<Tuple, 2>& input_simplices,
        const Tuple& final_simplex) override;


    void set_collapse_func(CollapseFuncType&& f);

    void set_collapse_type(OpType t);

private:
    wmtk::attribute::MeshAttributeHandle<T> m_handle;
    OpType m_optype = std::is_same_v<T, double> ? OpType::Mean : OpType::CopyTuple;
    CollapseFuncType m_collapse_op;
};


template <typename T>
BasicCollapseNewAttributeStrategy(const wmtk::attribute::MeshAttributeHandle<T>&)
    -> BasicCollapseNewAttributeStrategy<T>;
} // namespace wmtk::operations::tri_mesh
