
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
    // using HandleVariant = std::variant<
    //    attribute::MeshAttributeHandle<double>,
    //    attribute::MeshAttributeHandle<int64_t>,
    //    attribute::MeshAttributeHandle<char>,
    //    attribute::MeshAttributeHandle<Rational>>;

    // if you map a j-simplex to a k-simplex then
    // if j < k: the system evaluates the function at every j-simplex that is a face of k
    // if k < j: the system evaluates the function at every j-simplex that is a coface of k
    // if j == k: the system assumes a point-wise update with no neighbors
    //
    static std::vector<Tuple> get_parent_simplices(
        const Mesh& m,
        const Mesh& parent,
        const simplex::Simplex& s,
        PrimitiveType parent_primitive_type);

    static std::vector<Tuple> get_parent_simplices(
        const attribute::MeshAttributeHandle& me,
        const attribute::MeshAttributeHandle& parent,
        const simplex::Simplex& s);

    virtual bool matches_attribute(
        const wmtk::attribute::MeshAttributeHandle& attr) const = 0;
    // const simplex::Simplex& s) const = 0;

    virtual void run(const simplex::Simplex& s) = 0;
    // virtual void update(const simplex::Simplex& simplex) = 0;
    //  placeholder for when this turns into a DAG that needs to be linearized
    //  virtual std::vector<HandleVariant> parent_handles() const = 0;

    virtual PrimitiveType primitive_type() const = 0;
    virtual Mesh& mesh() = 0;
};


} // namespace wmtk::operations
