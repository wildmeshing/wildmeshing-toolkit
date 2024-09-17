
#pragma once
#include <variant>
//#include <wmtk/utils/Rational.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/operations/AttributeTransferEdge.hpp>
// #include "NewAttributeStrategy.hpp"

namespace wmtk {
class Mesh;
namespace simplex {
class Simplex;
}
} // namespace wmtk

namespace wmtk::operations {


class AttributeTransferStrategyBase : public AttributeTransferEdge
{
public:
    AttributeTransferStrategyBase(const attribute::MeshAttributeHandle& my_handle);
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

    const attribute::MeshAttributeHandle& handle() const { return m_handle; }
    attribute::MeshAttributeHandle& handle() { return m_handle; }

    std::vector<wmtk::attribute::MeshAttributeHandle> targets() const final override
    {
        return {handle()};
    }


    // virtual bool run(const simplex::Simplex& s)  = 0;
    bool matches_attribute(const wmtk::attribute::MeshAttributeHandle& attr) const;

    // const simplex::Simplex& s) const = 0;

    virtual void run(const simplex::Simplex& s) const = 0;
    // virtual void update(const simplex::Simplex& simplex) = 0;
    //  placeholder for when this turns into a DAG that needs to be linearized
    //  virtual std::vector<HandleVariant> parent_handles() const = 0;

    virtual PrimitiveType primitive_type() const = 0;
    virtual Mesh& mesh() = 0;
    const Mesh& mesh() const;

    // runs the transfer on every simplex - good for initializing an attribute that will be
    // managed by transfer
    void run_on_all() const;

private:
    attribute::MeshAttributeHandle m_handle;
};


} // namespace wmtk::operations
