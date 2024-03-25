#pragma once

#include <memory>
namespace wmtk {
class Mesh;
}

namespace wmtk::io {


// temporary hack for the deadline. In the future meshes need to store their own names if they want
// to be accessed by name
class CachedMultiMesh
{
public:
    CachedMultiMesh(
        const std::string& name,
        std::map<std::string, std::vector<int64_t>> multimesh_names,
        std::shared_ptr<Mesh> root = nullptr);
    ~CachedMultiMesh();

    void load(const std::filesystem::path& path);
    void load(std::shared_ptr<Mesh> root);
    void flush();
    // passed in mesh.uv
    std::shared_ptr<Mesh> get_from_path(const std::string& name);


    std::shared_ptr<Mesh> get(const std::string& name);
    std::shared_ptr<Mesh> get_root();

    const std::vector<int64_t>& get_id(const std::string& name) const;
    const std::vector<int64_t>& get_id_from_path(const std::string& name) const;

    const std::map<std::string, std::vector<int64_t>>& get_multimesh_names() const;


    CachedMultiMesh(CachedMultiMesh&&);
    CachedMultiMesh& operator=(CachedMultiMesh&&);

private:
    std::string m_name;
    std::shared_ptr<Mesh> m_root;
    // "uv" "position"
    // "mesh.uv" "mesh.position"
    std::map<std::string, std::vector<int64_t>> m_multimesh_names;
};


} // namespace wmtk::io
