#pragma once
#include <map>
#include "NamedMultiMesh.hpp"

namespace wmtk::components::input {

    class InputOptions;
class MeshCollection
{
public:
    NamedMultiMesh& add_mesh(NamedMultiMesh o);
    NamedMultiMesh& add_mesh(const InputOptions& opts);

    const NamedMultiMesh& get_named_multimesh(const std::string_view& path) const;
    const Mesh& get_mesh(const std::string_view& path) const;

    NamedMultiMesh& get_named_multimesh(const std::string_view& path);
    Mesh& get_mesh(const std::string_view& path);



private:
    std::map<std::string_view, NamedMultiMesh> m_meshes;
};
} // namespace wmtk::components::input
