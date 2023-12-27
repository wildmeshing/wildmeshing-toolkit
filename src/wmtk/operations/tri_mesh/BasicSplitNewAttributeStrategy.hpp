#pragma once
#include <functional>
#include "SplitNewAttributeStrategy.hpp"

namespace wmtk::operations::tri_mesh {

template <typename T>
class BasicSplitNewAttributeStrategy : public SplitNewAttributeStrategy
{
public:
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

    void set_split_rib_strategy(SplitRibFuncType&& f);
    void set_split_strategy(SplitFuncType&& f);

    void set_standard_split_rib_strategy(SplitRibBasicStrategy t) override;
    void set_standard_split_strategy(SplitBasicStrategy t) override;

    Mesh& mesh() override;
    PrimitiveType primitive_type() const override;
    void update_handle_mesh(Mesh& m) override;
    bool matches_attribute(const attribute::MeshAttributeHandleVariant&) const override;


private:
    wmtk::attribute::MeshAttributeHandle<T> m_handle;
    SplitRibFuncType m_split_rib_op;
    SplitFuncType m_split_op;
};


template <typename T>
BasicSplitNewAttributeStrategy(const wmtk::attribute::MeshAttributeHandle<T>&)
    -> BasicSplitNewAttributeStrategy<T>;
} // namespace wmtk::operations::tri_mesh
