#include "DifferentiableFunction.hpp"
using namespace wmtk;
using namespace wmtk::function;

DifferentiableFunction::DifferentiableFunction(
    const Mesh& mesh,
    const MeshAttributeHandle<double>& vertex_attribute_handle)
    : Function(mesh)
    , m_vertex_attribute_handle(vertex_attribute_handle){};
