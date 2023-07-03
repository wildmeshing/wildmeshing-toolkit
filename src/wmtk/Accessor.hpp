#pragma once

#include <memory>
#include <type_traits>
#include "AccessorAccessMode.hpp"
#include "AccessorBase.hpp"
#include "AttributeHandle.hpp"
#include "Tuple.hpp"

#include <Eigen/Dense>

namespace wmtk {
class Mesh;
class TriMesh;
class TetMesh;

template <typename T>
class AccessorCache;

template <typename T, bool IsConst = false>
class Accessor : public AccessorBase<T, IsConst>
{
public:
    friend class Mesh;
    friend class TriMesh;
    friend class TetMesh;
    using BaseType = AccessorBase<T, IsConst>;
    using MeshType = typename BaseType::MeshType; // const correct Mesh object

    using MapResult = typename BaseType::MapResult; // Eigen::Map<VectorX<T>>
    using ConstMapResult = typename BaseType::ConstMapResult; // Eigen::Map<const VectorX<T>>


    using MapResultT = typename BaseType::MapResultT; // MapResult or ConstMapResult for constness
    using TT = typename BaseType::TT; // T or T& for const correctness


    Accessor(
        MeshType& m,
        const MeshAttributeHandle<T>& handle,
        AccessorAccessMode read_access_mode = AccessorAccessMode::Immediate,
        AccessorAccessMode write_access_mode = IsConst ? AccessorAccessMode::None
                                                       : AccessorAccessMode::Immediate);

    ~Accessor();

    AccessorAccessMode read_access_mode() const;
    AccessorAccessMode write_access_mode() const;

    ConstMapResult vector_attribute(const Tuple& t) const;
    MapResultT vector_attribute(const Tuple& t);

    T scalar_attribute(const Tuple& t) const;
    TT scalar_attribute(const Tuple& t);

    // returns the size of the underlying attribute

    using BaseType::size; // const() -> long
    using BaseType::stride; // const() -> long

protected:
    using BaseType::set_attribute; // (const vector<T>&) -> void

private:
    AccessorAccessMode m_read_mode;
    AccessorAccessMode m_write_mode;

    std::unique_ptr<AccessorCache<T>> m_cache;
};

// template <typename T>
// Accessor(Mesh& mesh, const MeshAttributeHandle<T>&) -> Accessor<T>;
// template <typename T>
// Accessor(const Mesh& mesh, const MeshAttributeHandle<T>&) -> Accessor<const T>;


template <typename T>
using ConstAccessor = Accessor<T, true>;
} // namespace wmtk
