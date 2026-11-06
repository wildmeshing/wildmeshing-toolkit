#pragma once

#include <wmtk/io/ComponentIO.hpp>

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <wmtk/Types.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/utils/FeatureInputs.hpp>

namespace wmtk::components::simwild {

struct InputData
{
    MatrixXd V_input;
    MatrixXr V_input_r;
    MatrixXi T_input;
    MatrixSi T_input_tag;
    std::vector<std::string> tag_names;

    /// Tagged faces of a tet mesh: rows of V_input, with one 0/1 column per face_tag_names
    /// entry. Empty when the mesh has no triangle block. In 2D, the tagged edges (two columns)
    /// of the curve inputs or of a tagged mesh's edge block, and face_tag_names their edge tags'
    /// names. A row's vertex order is its orientation.
    MatrixXi F_tagged;
    MatrixSi F_tags;
    std::vector<std::string> face_tag_names;
    /// With WMTK_PER_INPUT_ORIENTATION: per row of F_tagged and column of F_tags, that tag's own
    /// orientation count along the row -- a tagged mesh's `orientation` (io::read_orientation()),
    /// a surface input's direction from the embedding. 0 rows otherwise, and then each tag of a
    /// row has +1 along it (io::orient_tagged_row()).
    MatrixSi F_orientation;

    MatrixXd V_envelope;
    MatrixXi F_envelope;
    /// Per row of F_envelope: the label bits of the input it came from, as the mesh's faces
    /// (2D: edges) carry them (SimWildMesh::init_label_envelopes(); 2D, SimWildMeshTri's).
    /// Empty when F_envelope is not the inputs'.
    std::vector<uint64_t> F_envelope_labels;
    /// Name of the physical group the envelope curves came from; empty when there was none.
    std::string envelope_name;
    /// An `envelope` input that is a labelled reference (io::is_labelled_envelope()), as tetwild,
    /// triwild and simwild hand theirs on: as given, for the run to hand on in turn while it holds
    /// to it. Its triangles (2D: segments) are then V_envelope and F_envelope, every row, and
    /// F_envelope_labels their labels' bits by name: a label is every bit of the mesh named as it
    /// (a face tag's, 2D an edge tag's, i; a region's, the face tags' count + j), 0 for a label the
    /// mesh does not have. Empty without one, and for an `envelope` without labels, held to as a
    /// whole as before the labels.
    std::optional<MeshData> envelope_reference;
    /// Its feature curves (3D, its edge block) of the labels the mesh's feature curves have: an
    /// input edge per edge and label, edge_input that label's feature input (feature_edge_names).
    utils::FeatureInputs reference_curves;
    /// Whether the input was simplified before the insertion (preserve_topology off). Around the
    /// input (envelope_reference "input") V_envelope is then the input as read, not what was
    /// inserted, and the mesh holds the input's own curves and curve points around the input
    /// (TetOptimizerMesh::InputCurves); around the simplified geometry it is the simplification.
    bool simplified_before_insertion = false;
    /// The diagonal eps_rel, eps_simplify_rel and length_rel are relative to (Parameters::init()):
    /// the inputs' own bounding box, the feature curves and points included, as in tetwild and
    /// triwild. -1 for a mesh input, whose own box is V_input's. Until 2026-11 SimWild used the
    /// box of the embedding, padded by a fifteenth of the diagonal on each side, for the optimizer,
    /// and the inputs' own for the simplification before the insertion: 1.22x apart on
    /// simwild_double_sphere_notop_3d, 1.42x on simwild_curves_2d.
    double diag = -1;

    /// The topological offset's correspondence handed on from an earlier offset (vertex field
    /// corr_input_vid, on both routes): per row of V_input, the V_input row of the input-complex
    /// vertex it was split off from, -1 for none. Empty when the mesh has no such field. Only the
    /// offset reads it, and only with save_offset_correspondence (see its spec).
    std::vector<int64_t> corr_input_vid;

    /// A surface input with a history to record (history_output): the files as read and what
    /// became of them before the arrangement, and the input triangles each surface facet of the
    /// embedding lies on, by its corners' positions sorted. Empty otherwise.
    bool has_input_history = false;
    history::SurfaceHistory input_history;
    std::vector<std::pair<std::array<std::array<double, 3>, 3>, std::vector<uint32_t>>>
        facet_sources;

    /// A surface input with curve or point inputs (feature curves and points): them as read,
    /// and where the insertion put them, in V_input's numbering. A tagged tet mesh with an edge
    /// block: its feature edges, one input edge per edge and tag (edge_input the tag's column),
    /// each placed on itself. Empty otherwise.
    utils::FeatureInputs features;
    utils::EmbedFeaturesResult features_placed;
    /// The name of each feature input, bit i of a feature edge's input set: the label of the i-th
    /// curve input, a tagged tet mesh's edge tags. Empty without feature curves.
    std::vector<std::string> feature_edge_names;
    /// A 2D curve input with point inputs: the row of V_input each point became, in
    /// input order. Empty otherwise.
    std::vector<size_t> input_point_vids;
    /// The input points' labels, which the mesh's point block writes (io::collect_point_tags()):
    /// per input point -- features.points in 3D, input_point_vids in 2D -- its label's column of
    /// point_names, the name of its input, or of a tagged mesh's point block's label.
    std::vector<int> input_point_labels;
    std::vector<std::string> point_names;
};

/**
 * @brief Read a .msh file that contains tags.
 */
/// A 2D mesh's label images -- integer cell fields tag_<k>, as img_to_msh_2d.py writes them --
/// as its region field, tags, when it has none: in place, both routes. `what` names the input in
/// errors.
void tags_from_label_images(MeshData& mesh, const std::string& what);
/// `envelope`, optional: an `envelope` input, on both routes read_mesh_data()'s.
InputData read_image_msh(const std::string& path, const MeshData* envelope = nullptr);
InputData read_mesh_data(const MeshData& mesh, const MeshData* envelope = nullptr);

/**
 * @brief Read one or multiple meshes and convert them into a tet mesh.
 *
 * Winding number is used to tag inside/outside. `features`, optional: feature curves and points
 * forced into the embedding with the surface (see InputData::features).
 */
InputData read_mesh(
    const std::vector<std::string>& input_paths,
    const std::string& output_filename,
    const nlohmann::json& json_params,
    const io::ComponentIO* component_io = nullptr,
    const utils::FeatureInputs* features = nullptr);

/**
 * @brief Read one or multiple 2D curve networks and convert them into a tagged triangle mesh.
 *
 * The 2D counterpart of read_mesh: exact arrangement of the curves, then a per-input winding
 * number to tag which faces belong to which material. See EmbedCurves.
 */
InputData read_curves(
    const std::vector<std::string>& input_paths,
    const std::string& output_filename,
    const nlohmann::json& json_params,
    const io::ComponentIO* component_io = nullptr,
    const utils::FeatureInputs* features = nullptr);

/**
 * @brief Decide whether an input is 2D or 3D.
 *
 * `dimension` is "auto" (the default), 2 or 3. Auto looks at the inputs:
 *   - a first input .msh (memory: tets, or tagged triangles) -> 0, the tagged route, whose
 *     reader decides from the content
 *   - any surface (io::ComponentIO::input_kind(): triangles) -> 3, for EmbedSurface
 *   - else curves                                           -> 2, for EmbedCurves
 * Points alone go with either.
 *
 * An explicit 2 or 3 forces the route; it does not skip the sniff, it disagrees with it
 * loudly, because the failure otherwise surfaces as a confusing parse error much later.
 */
int resolve_input_dimension(
    const std::vector<std::string>& input_paths,
    const nlohmann::json& json_params,
    const io::ComponentIO* component_io = nullptr);

} // namespace wmtk::components::simwild