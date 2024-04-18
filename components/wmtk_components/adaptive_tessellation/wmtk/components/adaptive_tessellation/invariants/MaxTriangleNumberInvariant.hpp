#pragma once

#include <Eigen/Dense>
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/AttributeHandle.hpp>
#include <wmtk/invariants/Invariant.hpp>
namespace wmtk {
class MaxTriangleNumberInvariant : public Invariant
{
    /**
     * Invariant for adaptive tessellation. The number of triangles in the mesh should not exceed
     * the maximum number of triangles
     */
public:
    MaxTriangleNumberInvariant(const Mesh& m, const int64_t max_triangle_number);
    bool before(const simplex::Simplex& t) const override;

public:
    const int64_t m_max_triangle_number;
};
} // namespace wmtk
