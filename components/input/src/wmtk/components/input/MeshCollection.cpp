
#include "MeshCollection.hpp"
#include "internal/split_path.hpp"


namespace wmtk::components::input {


void MeshCollection::add_mesh(NamedMultiMesh m)
{
    m_meshes.emplace(m.root_name(), std::move(m));
}

const NamedMultiMesh& MeshCollection::get_named_multimesh(const std::string_view& path) const
{
    using namespace std;
    const std::string_view nmm_name = *internal::split_path(path).begin();
    return m_meshes.at(nmm_name);
}
const Mesh& MeshCollection::get_mesh(const std::string_view& path) const
{
    return get_named_multimesh(path).get_mesh(path);
}
} // namespace wmtk::components::input
