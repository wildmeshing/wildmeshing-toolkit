#pragma once

#include <wmtk/TriMeshOperation.h>
namespace wmtk {
template <
    typename MeshType,
    typename DerivedOperationType,
    typename BaseOperationType = TriMeshOperation
#if defined(__cpp_concepts)
    >
    requires(
        std::is_base_of_v<TriMesh, MeshType> &&
        std::is_base_of_v<TriMeshOperation, BaseOperationType>)
#else
    ,
    typename = std::enable_if_t<std::is_base_of_v<TriMesh, MeshType>, void>,
    typename = std::enable_if_t<std::is_base_of_v<TriMeshOperation, BaseOperationType>, void>>
#endif
class TriMeshOperationShim : public BaseOperationType
{
public:
    using Tuple = TriMeshOperation::Tuple;
    DerivedOperationType& derived() { return static_cast<DerivedOperationType&>(*this); }
    const DerivedOperationType& derived() const
    {
        return static_cast<const DerivedOperationType&>(*this);
    }

    bool execute(TriMesh& m, const Tuple& t) override
    {
        return execute(static_cast<MeshType&>(m), t);
    }
    bool before(TriMesh& m, const Tuple& t) override
    {
        return before(static_cast<MeshType&>(m), t);
    }
    bool after(TriMesh& m) override { return after(static_cast<MeshType&>(m)); }

private:
    bool execute(MeshType& m, const Tuple& t) { return derived().execute(m, t); }
    bool before(MeshType& m, const Tuple& t) { return derived().before(m, t); }
    bool after(MeshType& m) { return derived().after(m); }
};
} // namespace wmtk
