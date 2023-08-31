#include "AreaAccuracy.hpp"
using namespace wmtk;
using namespace wmtk::energy;

AreaAccuracy::AreaAccuracy(const TriMesh& mesh1, const TriMesh& mesh2)
    : DifferentiableEnergy(mesh1)
    , m_position_mesh(mesh2)
    , m_3d_position_handle(mesh2.get_attribute_handle<double>("position", PrimitiveType::Vertex))
{}
