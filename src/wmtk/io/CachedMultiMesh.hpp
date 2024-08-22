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

    /**
     * @brief Load a mesh from file and make it the root mesh.
     */
    void load(const std::filesystem::path& path);
    void load(std::shared_ptr<Mesh> root);

    /**
     * @brief Sets m_root to nullptr.
     *
     * But why?
     */
    void flush();

    /**
     * @brief Get the mesh with the given name.
     *
     * If the mesh is a child mesh, use the notation: "root.child".
     *
     * @param Mesh name following the convention "root.child".
     */
    std::shared_ptr<Mesh> get_from_path(const std::string& name);

    /**
     * @brief Get the mesh with the given name.
     *
     * Returns nullptr if the mesh is not in m_multimesh_names.
     *
     * @param Mesh name (without dot notation)
     */
    std::shared_ptr<Mesh> get(const std::string& name);

    /**
     * @brief Get the root mesh.
     */
    std::shared_ptr<Mesh> get_root();

    /**
     * @brief Get the ID of the mesh.
     *
     * What ID? I don't know for sure but I assume it's the multimesh ID.
     */
    const std::vector<int64_t>& get_id(const std::string& name) const;

    /**
     * @brief Gets the ID of the mesh with the given name.
     *
     * If the mesh is a child mesh, use the notation: "root.child".
     *
     * @param Mesh name following the convention "root.child".
     */
    const std::vector<int64_t>& get_id_from_path(const std::string& name) const;

    /**
     * @brief Returns m_multimesh_names.
     */
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
