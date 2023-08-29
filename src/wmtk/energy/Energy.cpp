#include "Energy.hpp"
namespace wmtk::energy {
Energy::Energy(Mesh& mesh)
    : m_mesh(mesh)
    , m_position_handle(m_mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex)){};

Energy::~Energy() = default;
} // namespace wmtk::energy
