
#include "MeshCollection.hpp"
#include "internal/split_path.hpp"
#include <wmtk/utils/Logger.hpp>

#include "input.hpp"

namespace wmtk::components::input {


NamedMultiMesh& MeshCollection::add_mesh(NamedMultiMesh m)
{
    auto [it, did] = m_meshes.emplace(m.root_name(), std::move(m));
    return it->second;
}

NamedMultiMesh& MeshCollection::add_mesh(const InputOptions& opts)
{
    return add_mesh(input(opts));
}


const NamedMultiMesh& MeshCollection::get_named_multimesh(const std::string_view& path) const
{
    using namespace std;
    const auto nmm_name = *internal::split_path(path).begin();
    if(nmm_name.empty() && m_meshes.size() == 1) {
        wmtk::logger().debug("MeshCollection accessed with an empty name, but has only 1 mesh so assuming that is the right mesh");
        return m_meshes.begin()->second;
    }
    return m_meshes.at(nmm_name);
}
const Mesh& MeshCollection::get_mesh(const std::string_view& path) const
{
    return get_named_multimesh(path).get_mesh(path);
}

Mesh& MeshCollection::get_mesh(const std::string_view& path) 
{
    return get_named_multimesh(path).get_mesh(path);
}
NamedMultiMesh& MeshCollection::get_named_multimesh(const std::string_view& path) 
{
    using namespace std;
    const std::string_view nmm_name = *internal::split_path(path).begin();
    if(nmm_name.empty() && m_meshes.size() == 1) {
        wmtk::logger().debug("MeshCollection accessed with an empty name, but has only 1 mesh so assuming that is the right mesh");
        return m_meshes.begin()->second;
    }
    return m_meshes.at(nmm_name);
}
} // namespace wmtk::components::input
