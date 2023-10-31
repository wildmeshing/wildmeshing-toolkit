#pragma once
#include <wmtk/operations/OperationFactory.hpp>
#include <wmtk/operations/tri_mesh/VertexSmoothUsingDifferentiableEnergy.hpp>

namespace wmtk::operations {
namespace tri_mesh {
class VertexSmoothUsingDifferentiableEnergy;
}

template <>
std::unique_ptr<Operation> OperationFactory<
    tri_mesh::VertexSmoothUsingDifferentiableEnergy>::create(wmtk::Mesh& m, const Tuple& t) const;
} // namespace wmtk::operations
