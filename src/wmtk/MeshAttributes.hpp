enum class PrimitiveType { Vertex, Edge, Triangle, Tetrahedron };

template <typename T>
class MeshAttributes
{
    MeshAttributes(const bool is_volume) { m_attributes.resize(is_volume ? 4 : 3); }

    AttributeHandle
    register_attribute(const std::string& name, const PrimitiveType& type, long size)
    {
        assert(m_handles.find(name) == m_handles.end());

        AttributeHandler handle;
        handle.type = type;
        handle.stride = size;

        const int index = handle.type_index();

        const auto& attr = m_attributes[index];

        handle.index = attr.size();
        m_handles[name] = handle;

        if (handle.index == 0) initial_stride = size;

        attr.emplace_back();
        if (handle.index > 0) {
            assert(initial_stride > 0);
            assert(attr.front().size() % initial_stride == 0);

            attr.back().resize((attr.front().size() / initial_stride) * size);
        }


        return handle;
    }

    AttributeHandle get_attribute_handle(const std::string& name) const
    {
        return m_handles.at(name);
    }

    Eigen::Map<T> get_attribute(const std::string& name, const Tuple& tuple) const
    {
        auto& handle = m_handles.at(name);
        return get_attribute(handle)
    }

    Eigen::Map<T> get_attribute(const AttributeHandle& handle, const Tuple& tuple)
    {
        const int index = handle.type_index();

        const auto& attrs = m_attributes[index];
        const auto& attr = attrs[handle.index];

        const long gid = tuple.id(handle.type);
        const long start = gid * handle.stride;

        return Eigen::Map<T>(attr.begin() + start, attr.begin() + start + handle.stride);
    }

    T get_single_attribute(const std::string& name, const Tuple& tuple) const
    {
        auto& handle = m_handles.at(name);
        return get_single_attribute(handle)
    }

    T get_single_attribute(const AttributeHandle& handle, const Tuple& tuple)
    {
        assert(handle.stride == 1);

        const int index = handle.type_index();

        const auto& attrs = m_attributes[index];
        const auto& attr = attrs[handle.index];

        const long gid = tuple.id(handle.type);

        return attr[gid];
    }


private:
    std::map<std::string, AttributeHandle> m_handles;
    long initial_stride = -1;

    std::vector<std::vector<std::vector<T>>> m_attributes;
};


class AttributeHandler
{
public:
    PrimitiveType type;
    long index;
    long stride;

    long type_index()
    {
        switch (type) {
        case PrimitiveType::Vertex: return 0;
        case PrimitiveType::Edge: return 1;
        case PrimitiveType::Triangle: return 2;
        case PrimitiveType::Tetrahedron: return 3;
        default: assert(false); break;
        }
    }
}