#pragma once

#include <wmtk/SurfaceTagAttributes.h>
#include <wmtk/MeshData.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/utils/EmbedTriangles.hpp>
#include <wmtk/utils/FeatureInputs.hpp>

#include <array>
#include <string>
#include <vector>

namespace wmtk::components::tetwild {

/**
 * @brief A tagged tet mesh as tetwild's tagged route takes it: the tets, and what tetwild holds
 * on them -- tagged faces, feature curves and points. No volumes: tetwild carries no tet tags
 * through its operations, and a mesh whose tets carry a label other than ambient is refused
 * (read_tagged_mesh()).
 *
 * The lower blocks are read as simwild's read_mesh_data() reads them, so a mesh means the same
 * to both: a triangle block is tagged faces, each label one input, every row oriented by its
 * vertex order; an edge block is feature curves, each label one feature input, every edge and
 * label an input edge placed on itself. A point block is input points, anchored where they are.
 */
struct TaggedMesh
{
    /// The vertices of the tets, renumbered in order: a vertex no tet has is not the mesh's.
    std::vector<Vector3d> V;
    /// The tets, positively oriented (read_tagged_mesh() refuses any other).
    std::vector<std::array<size_t, 4>> T;
    /// The tagged faces, rows of the triangle block on V, in the block's vertex order, which is
    /// their orientation; F_sets[r] has bit j for label face_names[j].
    std::vector<std::array<size_t, 3>> F;
    std::vector<InputSet> F_sets;
    std::vector<std::string> face_names;
    /// The edge block: per edge and label, an input edge (features.edges, edge_input the
    /// label's column of feature_names) placed on itself (features_placed.edge_tiling). The
    /// point block: features.points, each the vertex features_placed.point_vertex names.
    utils::FeatureInputs features;
    utils::EmbedFeaturesResult features_placed;
    std::vector<std::string> feature_names;
};

/// Whether `mesh` has tets: the tagged route's input, rather than a surface.
bool is_tet_mesh(const MeshData& mesh);

/**
 * @brief Read a tagged tet mesh for tetwild, throwing before any work on what tetwild cannot
 * hold: tets tagged other than ambient (a volume, simwild's to remesh), cells other than tets,
 * triangles, edges and points, tets not positively oriented, a lower block without tags, a row
 * without a tag, rows labelled EnvelopeSurface (an embedded envelope), more labels than
 * SurfaceTagAttributes::max_inputs, a row on a vertex no tet has. `what` names the input in the
 * errors.
 *
 * Fields: `tags` is read, and every other field dropped (io::ComponentIO::data_fields), on both
 * routes: a result, tetwild's own fields and all, goes back in as it is.
 */
TaggedMesh read_tagged_mesh(const MeshData& mesh, const std::string& what);

} // namespace wmtk::components::tetwild
