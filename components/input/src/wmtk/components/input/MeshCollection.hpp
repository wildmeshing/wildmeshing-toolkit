#pragma once
#include <map>
#include "NamedMultiMesh.hpp"

namespace wmtk::components::input {

class MeshCollection
{
public:
    void add_mesh(NamedMultiMesh o);

    const NamedMultiMesh& get_named_multimesh(const std::string_view& path) const;
    const Mesh& get_mesh(const std::string_view& path) const;


private:
    std::map<std::string_view, NamedMultiMesh> m_meshes;
};
} // namespace wmtk::components::input
