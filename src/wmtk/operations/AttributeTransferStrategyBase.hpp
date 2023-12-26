
#pragma once
#include <variant>
//#include <wmtk/utils/Rational.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
// #include "NewAttributeStrategy.hpp"

namespace wmtk {
    class Mesh;
}

namespace wmtk::operations {


class AttributeTransferStrategyBase
{
public:
    AttributeTransferStrategyBase() = default;
    virtual ~AttributeTransferStrategyBase();
    // placeholder for when this turns into a DAG that needs to be linearized
    //using HandleVariant = std::variant<
    //    attribute::MeshAttributeHandle<double>,
    //    attribute::MeshAttributeHandle<long>,
    //    attribute::MeshAttributeHandle<char>,
    //    attribute::MeshAttributeHandle<Rational>>;

    // if you map a j-simplex to a k-simplex then
    // if j < k: the system evaluates the function at every j-simplex that is a face of k
    // if k < j: the system evaluates the function at every j-simplex that is a coface of k
    // if j == k: the system assumes a point-wise update with no neighbors
    //
    static std::vector<Tuple>
    get_parent_simplices(const Mesh& m, const Mesh& parent, const Simplex& s, PrimitiveType parent_primitive_type);

    template <typename A, typename B>
    static std::vector<Tuple>
    get_parent_simplices(const MeshAttributeHandle<A>& me, const MeshAttributeHandle<B>& parent, const Simplex& s);

    // placeholder for when this turns into a DAG that needs to be linearized
    // virtual std::vector<HandleVariant> parent_handles() const = 0;

    virtual PrimitiveType primitive_type() const = 0;
    virtual Mesh& mesh() = 0;
};

    template <typename A, typename B>
    std::vector<Tuple>
    AttributeTransferStrategyBase::get_parent_simplices(const MeshAttributeHandle<A>& me, const MeshAttributeHandle<B>& parent, const Simplex& s) {
        return get_parent_simplices(me.mesh(), parent.mesh(), s, parent.primitive_type());
    }
} // namespace wmtk::operations
