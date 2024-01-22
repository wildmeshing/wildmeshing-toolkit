#pragma once
#include <wmtk/Mesh.hpp>
#include <wmtk/io/MeshReader.hpp>

namespace wmtk::io {


// temporary hack for the deadline. In the future meshes need to store their own names if they want
// to be accessed by name
class CachedMultiMesh
{
public:
    CachedMultiMesh(
        const std::string& name,
        std::map<std::string, std::vector<int64_t>> multimesh_names,
        std::shared_ptr<Mesh> root = nullptr)
        : m_name(name)
        , m_root(root)
        , m_multimesh_names(std::move(multimesh_names))
    {}

    void load(const std::filesystem::path& path);
    void load(std::shared_ptr<Mesh> root) { m_root = root; }
    void flush() { m_root = nullptr; }
    // passed in mesh.uv
    std::shared_ptr<Mesh> get_from_path(const std::string& name)
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


    std::shared_ptr<Mesh> get(const std::string& name)
    {
        return m_root->get_multi_mesh_mesh(m_multimesh_names.at(name)).shared_from_this();
    }
    std::shared_ptr<Mesh> get_root() { return m_root; }

    const std::vector<int64_t>& get_id(const std::string& name) const
    {
        return m_multimesh_names.at(name);
    }
    const std::vector<int64_t>& get_id_from_path(const std::string& name) const
    {
        // double checks that this starts with //mesh.
        assert(name.substr(0, name.find('.')) == m_name);

        // extracts uv and calls get
        return get_id(name.substr(m_name.find('.') + 1));
    }

private:
    std::string m_name;
    std::shared_ptr<Mesh> m_root;
    // "uv" "position"
    // "mesh.uv" "mesh.position"
    std::map<std::string, std::vector<int64_t>> m_multimesh_names;
};


inline void CachedMultiMesh::load(const std::filesystem::path& path)
{
    m_root = wmtk::read_mesh(path);
    // for (const auto& pr : m_multimesh_names) {
    //     const auto& mm_name = pr.first;
    //     const auto& id = pr.second;
    //     map[mm_name] = mesh->get_multi_mesh_mesh(id).shared_from_this();
    // }
}
} // namespace wmtk::io
