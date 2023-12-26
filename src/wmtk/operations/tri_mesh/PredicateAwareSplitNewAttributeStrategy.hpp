
#pragma once
#include <bitset>
#include <functional>
#include "SplitNewAttributeStrategy.hpp"

namespace wmtk::operations::tri_mesh {

template <typename T>
class PredicateAwareSplitNewAttributeStrategy : public SplitNewAttributeStrategy
{
public:
    using VecType = VectorX<T>;
    using SplitRibFuncType =
        std::function<VecType(const VecType&, const VecType&, const std::bitset<2>&)>;
    using SplitFuncType =
        std::function<std::array<VecType, 2>(const VecType&, const std::bitset<2>&)>;

    using SimplexPredicateType = std::function<bool(const simplex::Simplex&)>;
    PredicateAwareSplitNewAttributeStrategy(wmtk::attribute::MeshAttributeHandle<T>& h);
    PredicateAwareSplitNewAttributeStrategy(
        const wmtk::attribute::MeshAttributeHandle<T>& h,
        Mesh& m);

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

    void set_simplex_predicate(SimplexPredicateType&& f);
    void set_standard_split_rib_strategy(SplitRibBasicStrategy t) override;
    void set_standard_split_strategy(SplitBasicStrategy t) override;
    void set_standard_simplex_predicate(BasicSimplexPredicate f);


    Mesh& mesh() override;
    PrimitiveType primitive_type() const override;
    void update_handle_mesh(Mesh& m) override;


    bool matches_attribute(const attribute::MeshAttributeHandleVariant&) const override;

private:
    wmtk::attribute::MeshAttributeHandle<T> m_handle;
    SplitRibFuncType m_split_rib_op;
    SplitFuncType m_split_op;
    SimplexPredicateType m_simplex_predicate;
};


template <typename T>
PredicateAwareSplitNewAttributeStrategy(const wmtk::attribute::MeshAttributeHandle<T>&)
    -> PredicateAwareSplitNewAttributeStrategy<T>;
} // namespace wmtk::operations::tri_mesh
