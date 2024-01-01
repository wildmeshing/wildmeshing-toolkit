#pragma once
#include <functional>
#include <wmtk/PrimitiveType.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/simplex/Simplex.hpp>

#include <wmtk/attribute/MeshAttributeHandle.hpp>

#include <wmtk/multimesh/operations/extract_operation_tuples.hpp>

namespace wmtk {
class Mesh;
class Tuple;
} // namespace wmtk

namespace wmtk::operations {

enum class BasicSimplexPredicate { Default, IsInterior, None };

class NewAttributeStrategy
{
public:
    template <typename T>
    using VecType = VectorX<T>;
    using SimplexPredicateType = std::function<bool(const simplex::Simplex&)>;

    using CollapseReturnData = wmtk::multimesh::operations::CollapseReturnData;
    using SplitReturnData = wmtk::multimesh::operations::SplitReturnData;
    using OperationTupleData = wmtk::multimesh::operations::OperationTupleData;


    virtual ~NewAttributeStrategy();

    virtual bool matches_attribute(const attribute::MeshAttributeHandleVariant&) const = 0;
    virtual void update_handle_mesh(Mesh&) = 0;

    virtual PrimitiveType primitive_type() const = 0;

    virtual Mesh& mesh() = 0;
    const Mesh& mesh() const;

    void set_simplex_predicate(SimplexPredicateType&& f);
    void set_simplex_predicate(BasicSimplexPredicate f);

    std::bitset<2> evaluate_predicate(PrimitiveType pt, const std::array<Tuple, 2>& simplices);

    virtual void update(const CollapseReturnData& ret_data, const OperationTupleData& tuples)
    {
        throw std::runtime_error("This is not a collapse");
    }

    virtual void update(const SplitReturnData& ret_data, const OperationTupleData& tuples)
    {
        throw std::runtime_error("This is not a split");
    }

protected:
private:
    SimplexPredicateType m_simplex_predicate = nullptr;
};

} // namespace wmtk::operations
