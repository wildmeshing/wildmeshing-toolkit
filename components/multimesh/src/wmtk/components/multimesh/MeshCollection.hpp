#pragma once
#include <map>
#include "NamedMultiMesh.hpp"
#include <memory>

namespace wmtk::components::multimesh {

    class InputOptions;
class MeshCollection
{
public:
    NamedMultiMesh& add_mesh(NamedMultiMesh o);
    //NamedMultiMesh& add_mesh(const InputOptions& opts);

    const NamedMultiMesh& get_named_multimesh(const std::string_view& path) const;
    const Mesh& get_mesh(const std::string_view& path) const;

    NamedMultiMesh& get_named_multimesh(const std::string_view& path);
    Mesh& get_mesh(const std::string_view& path);


    std::map<std::string, std::shared_ptr<const Mesh>> all_meshes() const;

private:
    std::map<std::string_view, std::unique_ptr<NamedMultiMesh>> m_meshes;
};
} // namespace wmtk::components::multimesh
