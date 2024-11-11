#pragma once
#include <map>
#include <memory>
#include "NamedMultiMesh.hpp"

namespace wmtk::components::multimesh {

class InputOptions;
class MeshCollection
{
public:
    NamedMultiMesh& add_mesh(NamedMultiMesh o);
    template <typename... Args>
    NamedMultiMesh& emplace_mesh(Args&&... args)
    {
        return add_mesh(NamedMultiMesh(std::forward<Args>(args)...));
    }
    // NamedMultiMesh& add_mesh(const InputOptions& opts);

    const NamedMultiMesh& get_named_multimesh(const std::string_view& path) const;
    const Mesh& get_mesh(const std::string_view& path) const;
    bool has_mesh(const std::string_view& path) const;
    bool has_named_multimesh(const std::string_view& path) const;

    NamedMultiMesh& get_named_multimesh(const std::string_view& path);
    Mesh& get_mesh(const std::string_view& path);

    // over time meshes can merge and have aliases
    // Thsi operation removes meshes whose naming structure aren't the root of a naming tree
    void make_canonical();

    std::map<std::string, const Mesh&> all_meshes() const;

private:
    std::map<std::string_view, std::unique_ptr<NamedMultiMesh>> m_meshes;
};
} // namespace wmtk::components::multimesh
