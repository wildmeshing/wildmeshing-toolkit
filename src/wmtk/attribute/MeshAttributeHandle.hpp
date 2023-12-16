#pragma once
#include "AttributeHandle.hpp"

namespace wmtk {
class Mesh;
}

namespace wmtk::attribute {

    /* @brief Handle that can construct an accessor on its own 
     * NOTE: This naming is inconsistent with the existing
     * AttributeHandle/MeshAttributeHandle nomenclature, but in the future most
     * applications should store SmartAttributeHandles instead of
     * MeshAttributeHandle, and after most of those changes are made we will
     * deprecate that name.
     */
template <typename T>
class SmartAttributeHandle
{
    SmartAttributeHandle(Mesh& m, const MeshAttributeHandle<T>&);
    SmartAttributeHandle(const SmartAttributeHandle<T>& o);
    SmartAttributeHandle(SmartAttributeHandle<T>&& o);
    SmartAttributeHandle<T>& operator=(const SmartAttributeHandle<T>& o);
    SmartAttributeHandle<T>& operator=(SmartAttributeHandle<T>&& o);

    operator MeshAttributeHandle<T>() const;

    const Mesh& mesh() const;
    const MeshAttributeHandle<T>& handle() const;

private:
    Mesh* m_mesh;
    MeshAttributeHandle<T> m_handle;
};

} // namespace wmtk::attribute

