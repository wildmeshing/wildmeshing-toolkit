
#include "MeshCollection.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include "internal/split_path.hpp"


namespace wmtk::components::multimesh {


NamedMultiMesh& MeshCollection::add_mesh(NamedMultiMesh&& m)
{
    auto mptr = std::make_unique<NamedMultiMesh>(std::move(m));
    auto [it, did] = m_meshes.emplace(mptr->root_name(), std::move(mptr));


    return *it->second;
}

// NamedMultiMesh& MeshCollection::add_mesh(const InputOptions& opts)
//{
//     return add_mesh(multimesh(opts));
// }


const Mesh& MeshCollection::get_mesh(const std::string_view& path) const
{
    return get_named_multimesh(path).get_mesh(path);
}

Mesh& MeshCollection::get_mesh(const std::string_view& path)
{
    return get_named_multimesh(path).get_mesh(path);
}
bool MeshCollection::has_named_multimesh(const std::string_view& path) const
{
    const std::string_view nmm_name = *internal::split_path(path).begin();
    if (nmm_name.empty() && m_meshes.size() == 1) {
        return true;
    }
    return m_meshes.find(nmm_name) != m_meshes.end();
}
bool MeshCollection::has_mesh(const std::string_view& path) const
{
    if (!has_named_multimesh(path)) {
        return false;
    } else {
        const auto& nmm = get_named_multimesh(path);
        return nmm.has_mesh(path);
    }
}

const NamedMultiMesh& MeshCollection::get_named_multimesh(const std::string_view& path) const
{
    assert(!m_meshes.empty());
    using namespace std;
#if defined(WMTK_ENABLED_CPP20)
    std::ranges::view auto split = internal::split_path(path);
#else
    auto split = internal::split_path(path);
#endif
    const auto nmm_name = *split.begin();
    if (nmm_name.empty() && m_meshes.size() == 1) {
        wmtk::logger().debug("MeshCollection accessed with an empty name, but has only 1 mesh so "
                             "assuming that is the right mesh");
        return *m_meshes.begin()->second;
    }
    if (auto it = m_meshes.find(nmm_name); it == m_meshes.end()) {
        std::vector<std::string_view> names;
        std::transform(
            m_meshes.begin(),
            m_meshes.end(),
            std::back_inserter(names),
            [](const auto& pr) { return pr.first; });
        wmtk::logger().error(
            "Was unable to find root mesh name {} among {} names [{}] in MeshCollection",
            nmm_name,
            m_meshes.size(),
            fmt::join(names, ","));
    }
    return *m_meshes.at(nmm_name);
}
NamedMultiMesh& MeshCollection::get_named_multimesh(const std::string_view& path)
{
    using namespace std;
    using namespace std;
#if defined(WMTK_ENABLED_CPP20)
    std::ranges::view auto split = internal::split_path(path);
#else
    auto split = internal::split_path(path);
#endif
    const auto nmm_name = *split.begin();
    if (nmm_name.empty() && m_meshes.size() == 1) {
        wmtk::logger().debug("MeshCollection accessed with an empty name, but has only 1 mesh so "
                             "assuming that is the right mesh");
        return *m_meshes.begin()->second;
    }
    try {
        return *m_meshes.at(nmm_name);
    } catch (const std::runtime_error& e) {
        wmtk::logger().warn("Failed to find mesh named {} in mesh list. Path was ", nmm_name, path);
        throw e;
    }
}
std::map<std::string, const Mesh&> MeshCollection::all_meshes() const
// std::map<std::string, std::shared_ptr<const Mesh>> MeshCollection::all_meshes() const
{
    // std::map<std::string, std::shared_ptr<const Mesh>> meshes;
    std::map<std::string, const Mesh&> meshes;
    for (const auto& [name, nnptr] : m_meshes) {
        const auto pr = nnptr->all_meshes();
        meshes.insert(pr.begin(), pr.end());
    }
    return meshes;
}

std::map<std::string, const Mesh&> MeshCollection::all_roots(bool only_true_roots) const
{
    // std::map<std::string, std::shared_ptr<const Mesh>> meshes;
    std::map<std::string, const Mesh&> meshes;
    for (const auto& [name, nnptr] : m_meshes) {
        const auto& mesh = nnptr->root();
        if (!only_true_roots || mesh.is_multi_mesh_root()) {
            meshes.emplace(name, mesh);
        }
    }
    return meshes;
}

void MeshCollection::make_canonical()
{
    for (auto it = m_meshes.begin(); it != m_meshes.end();) {
        if (it->second->root().is_multi_mesh_root()) {
            ++it;
        } else {
            it = m_meshes.erase(it);
        }
    }
}
bool MeshCollection::is_valid(bool pass_exceptions) const
{
    for (const auto& [name, nmmptr] : m_meshes) {
        if (!nmmptr->is_valid(pass_exceptions)) {
            return false;
        }
    }
    return true;
}

// std::vector<const NamedMultiMesh*> get_named_multimeshes(const Mesh& m) const {

//    for(const& [_, nmm]: m_meshes) {
//        if(m.is_string
//    }
//}
} // namespace wmtk::components::multimesh
