#pragma once

#include <wmtk/MeshData.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>

namespace wmtk {

/**
 * @brief A target edge length that varies over space: the `sizing_mesh` option of tetwild,
 * triwild, simwild and isotropic_remeshing.
 *
 * A mesh whose vertices carry the absolute target length as the vertex field `target_length`
 * (one positive value per vertex), interpolated linearly inside its cells:
 *
 * - Planar: its triangles, read in the xy plane. A point outside every triangle is not covered.
 * - Volume: its tetrahedra. A point outside every tet is not covered.
 * - Surface: its triangles in 3D, at the point of the surface closest to the query. Every point
 *   is covered.
 *
 * The queries only read, so threads may share one.
 */
class SizingMesh
{
public:
    enum class Kind { Planar, Volume, Surface };

    /// The vertex field holding the lengths.
    static constexpr const char* field_name = "target_length";

    /**
     * @brief Reads `mesh`'s cells of `kind` and its field `target_length`.
     *
     * Throws when the mesh has no such cells or no such field, when the field is not one value
     * per vertex or a value is not positive, or when a cell refers to a missing vertex. `option`
     * names the option in the error messages.
     */
    SizingMesh(const MeshData& mesh, Kind kind, const std::string& option = "sizing_mesh");
    ~SizingMesh();
    SizingMesh(const SizingMesh&) = delete;
    SizingMesh& operator=(const SizingMesh&) = delete;

    Kind kind() const;

    /// The target length at `p`, a Planar mesh only: positive where covered, -1 where not.
    double operator()(const Eigen::Vector2d& p) const;
    /// The target length at `p`, a Volume or Surface mesh: positive where covered, -1 where not.
    double operator()(const Eigen::Vector3d& p) const;

    double min_length() const;
    double max_length() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace wmtk
