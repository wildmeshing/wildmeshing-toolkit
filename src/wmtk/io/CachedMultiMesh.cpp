#include <wmtk/Mesh.hpp>
#include <wmtk/io/MeshReader.hpp>

#include "CachedMultiMesh.hpp"
namespace wmtk::io {


CachedMultiMesh::CachedMultiMesh(
    const std::string& name,
    std::map<std::string, std::vector<int64_t>> multimesh_names,
    std::shared_ptr<Mesh> root)
    : m_name(name)
    , m_root(root)
    , m_multimesh_names(std::move(multimesh_names))
{}
CachedMultiMesh::~CachedMultiMesh() = default;


void CachedMultiMesh::load(const std::filesystem::path& path)
{
    load(wmtk::read_mesh(path));
}

void CachedMultiMesh::load(std::shared_ptr<Mesh> root)
{
    m_root = root;
}
void CachedMultiMesh::flush()
{
    m_root = nullptr;
}
// passed in mesh.uv
std::shared_ptr<Mesh> CachedMultiMesh::get_from_path(const std::string& name)
{
    auto dot_it = name.find('.');
    // double checks that this starts with //mesh.
    assert(name.substr(0, dot_it) == m_name);
    if (dot_it == std::string::npos) {
        return get_root();
    }

    // extracts uv and calls get
    return get(name.substr(dot_it + 1));
}


std::shared_ptr<Mesh> CachedMultiMesh::get(const std::string& name)
{
    return m_root->get_multi_mesh_mesh(m_multimesh_names.at(name)).shared_from_this();
}
std::shared_ptr<Mesh> CachedMultiMesh::get_root()
{
    return m_root;
}
namespace {
const static std::vector<int64_t> root_id = {};
}

const std::vector<int64_t>& CachedMultiMesh::get_id(const std::string& name) const
{
    if (name.empty()) {
        return root_id;
    }
    if (!name.empty() && m_multimesh_names.find(name) == m_multimesh_names.end()) {
        throw std::runtime_error(fmt::format(
            "Could not find named multimesh {} in {}. Key {} was not among [{}]",
            name,
            m_name,
            name,
            "hi"));
    }
    return root_id;
}
const std::vector<int64_t>& CachedMultiMesh::get_id_from_path(const std::string& name) const
{
    const auto idx = name.find('.');
    // double checks that this starts with //mesh.
    assert(name.substr(0, idx) == m_name);

    if (idx == std::string::npos) {
        return get_id("");
    }
    const std::string subname = name.substr(idx + 1);
    // extracts uv and calls get
    return get_id(subname);
}

CachedMultiMesh::CachedMultiMesh(CachedMultiMesh&&) = default;
CachedMultiMesh& CachedMultiMesh::operator=(CachedMultiMesh&&) default;

} // namespace wmtk::io
