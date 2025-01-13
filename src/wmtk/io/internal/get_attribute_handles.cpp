#include <vector>
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/io/MeshWriter.hpp>

namespace wmtk::io::internal {

namespace {
class AttributeGetter : public MeshWriter
{
public:
    const Mesh& m_root;
    AttributeGetter(const Mesh& m)
        : m_root(m.get_multi_mesh_root())
        , current_mesh(m.absolute_multi_mesh_id())
    {}
    /*These are useless, just override*/
    void write_top_simplex_type(const PrimitiveType type) override {}
    void write_absolute_id(const std::vector<int64_t>& id) override { current_mesh = id; }
    void write_capacities(const std::vector<int64_t>& capacities) override {}
    bool write(const int dim) override { return true; }
    /**/

    template <typename T>
    void write(const std::string& name, const int8_t type)
    {
        const auto& mesh = m_root.get_multi_mesh_mesh(current_mesh);
        attributes[current_mesh].emplace_back(
            mesh.get_attribute_handle<T>(name, get_primitive_type_from_id(type)));
    }

    void write(
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const std::vector<char>& val,
        const char default_val) override
    {
        write<std::decay_t<decltype(default_val)>>(name, type);
    }

    void write(
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const std::vector<int64_t>& val,
        const int64_t default_val) override
    {
        write<std::decay_t<decltype(default_val)>>(name, type);
    }

    void write(
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const std::vector<double>& val,
        const double default_val) override
    {
        write<std::decay_t<decltype(default_val)>>(name, type);
    }

    void write(
        const std::string& name,
        const int64_t type,
        const int64_t stride,
        const std::vector<Rational>& val,
        const Rational& default_val) override
    {
        write<std::decay_t<decltype(default_val)>>(name, type);
    }
    std::map<std::vector<int64_t>, std::vector<wmtk::attribute::MeshAttributeHandle>> attributes;
    std::vector<int64_t> current_mesh;
};

} // namespace

std::vector<wmtk::attribute::MeshAttributeHandle> get_attribute_handles(const Mesh& m)
{
    AttributeGetter ag(m);
    m.serialize(ag);
    return ag.attributes[m.absolute_multi_mesh_id()];
}
} // namespace wmtk::io::internal
