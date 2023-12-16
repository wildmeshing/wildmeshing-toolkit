#pragma once
#include "AttributeHandle.hpp"

namespace wmtk {
class Mesh;
}

namespace wmtk::attribute {

template <typename T>
class SmartAttributeHandle
{
    SmartAttributeHandle(Mesh& m, const MeshAttributeHandle<T>&);
    SmartAttributeHandle(const SmartAttributeHandle<T>& o);
    SmartAttributeHandle(SmartAttributeHandle<T>&& o);
    SmartAttributeHandle<T>& operator=(const SmartAttributeHandle<T>& o);
    SmartAttributeHandle<T>& operator=(SmartAttributeHandle<T>&& o);


    const Mesh& mesh() const;
    const MeshAttributeHandle<T>& handle() const;

private:
    Mesh* m_mesh;
    MeshAttributeHandle<T> m_handle;
};

} // namespace wmtk::attribute

