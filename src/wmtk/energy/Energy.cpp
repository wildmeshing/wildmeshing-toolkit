#include "Energy.hpp"

Energy::Energy(const Mesh& mesh)
    : m_mesh(mesh)
    , m_position_handle(m_mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex)){};

Energy::Energy(const Mesh& mesh, const MeshAttributeHandle<double>& position_handle)
    : m_mesh(mesh)
    , m_position_handle(position_handle){};
