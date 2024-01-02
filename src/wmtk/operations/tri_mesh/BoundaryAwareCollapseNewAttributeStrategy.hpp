#pragma once
#include "CollapseNewAttributeStrategy.hpp"

namespace wmtk::operations::tri_mesh {

template <typename T>
class BoundaryAwayCollapseNewAttributeStrategy : public CollapseNewAttributeStrategy
{
public:
    using VecType = VectorX<T>;
    using CollapseFuncType = std::function<VecType(const VecType&, const VecType&)>;
    BoundaryAwayCollapseNewAttributeStrategy(wmtk::attribute::MeshAttributeHandle<T>& h);

    void assign_collapsed(
        PrimitiveType pt,
        const std::array<Tuple, 2>& input_simplices,
        const Tuple& final_simplex) override;


    void set_collapse_strategy(CollapseFuncType&& f);
    void set_standard_collapse_strategy(CollapseBoundaryAwayStrategy t) override;


    Mesh& mesh() override;
    PrimitiveType primitive_type() const override;

    void update_handle_mesh(Mesh& m) override;
    bool matches_attribute(const attribute::MeshAttributeHandleVariant&) const override;


private:
    wmtk::attribute::MeshAttributeHandle<T> m_handle;
    CollapseFuncType m_collapse_op;
};


template <typename T>
BoundaryAwayCollapseNewAttributeStrategy(const wmtk::attribute::MeshAttributeHandle<T>&)
    -> BoundaryAwayCollapseNewAttributeStrategy<T>;
} // namespace wmtk::operations::tri_mesh
