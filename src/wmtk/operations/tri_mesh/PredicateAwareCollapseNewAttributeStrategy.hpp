
#pragma once
#include <bitset>
#include "CollapseNewAttributeStrategy.hpp"

namespace wmtk::operations::tri_mesh {

template <typename T>
class PredicateAwareCollapseNewAttributeStrategy : public CollapseNewAttributeStrategy
{
public:
    using VecType = VectorX<T>;
    using CollapseFuncType =
        std::function<VecType(const VecType&, const VecType&, const std::bitset<2>&)>;
    using SimplexPredicateType = std::function<bool(const simplex::Simplex&)>;

    PredicateAwareCollapseNewAttributeStrategy(wmtk::attribute::MeshAttributeHandle<T>& h);
    PredicateAwareCollapseNewAttributeStrategy(
        const wmtk::attribute::MeshAttributeHandle<T>& h,
        Mesh& m);

    void assign_collapsed(
        PrimitiveType pt,
        const std::array<Tuple, 2>& input_simplices,
        const Tuple& final_simplex) override;


    void set_collapse_strategy(CollapseFuncType&& f);
    void set_simplex_predicate(SimplexPredicateType&& f);
    void set_standard_collapse_strategy(CollapseBasicStrategy t) override;
    void set_standard_simplex_predicate(BasicSimplexPredicate f);


    Mesh& mesh() override;
    PrimitiveType primitive_type() const override;

    void update_handle_mesh(Mesh& m) override;
    bool matches_attribute(const attribute::MeshAttributeHandleVariant&) const override;

private:
    wmtk::attribute::MeshAttributeHandle<T> m_handle;
    CollapseFuncType m_collapse_op;
    SimplexPredicateType m_simplex_predicate;
};


template <typename T>
PredicateAwareCollapseNewAttributeStrategy(const wmtk::attribute::MeshAttributeHandle<T>&)
    -> PredicateAwareCollapseNewAttributeStrategy<T>;
} // namespace wmtk::operations::tri_mesh
