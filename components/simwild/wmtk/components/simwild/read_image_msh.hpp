#pragma once

#include <wmtk/io/ComponentIO.hpp>

#include <nlohmann/json.hpp>
#include <string>
#include <wmtk/Types.hpp>
#include <wmtk/history/History.hpp>

namespace wmtk::components::simwild {

struct InputData
{
    MatrixXd V_input;
    MatrixXr V_input_r;
    MatrixXi T_input;
    MatrixSi T_input_tag;
    std::vector<std::string> tag_names;

    /// Tagged faces of a tet mesh: rows of V_input, with one 0/1 column per face_tag_names
    /// entry. Empty when the mesh has no triangle block.
    MatrixXi F_tagged;
    MatrixSi F_tags;
    std::vector<std::string> face_tag_names;

    MatrixXd V_envelope;
    MatrixXi F_envelope;
    /// Per row of F_envelope: the label bits of the input it came from, as the mesh's faces
    /// carry them (SimWildMesh::m_input_envelopes). Empty when F_envelope is not the inputs'.
    std::vector<uint64_t> F_envelope_labels;
    /// Name of the physical group the envelope curves came from; empty when there was none.
    std::string envelope_name;

    /// A surface input with a history to record (history_output): the files as read and what
    /// became of them before the arrangement, and the input triangles each surface facet of the
    /// embedding lies on, by its corners' positions sorted. Empty otherwise.
    bool has_input_history = false;
    history::SurfaceHistory input_history;
    std::vector<std::pair<std::array<std::array<double, 3>, 3>, std::vector<uint32_t>>>
        facet_sources;
};

/**
 * @brief Read a .msh file that contains tags.
 */
InputData read_image_msh(const std::string& path);
InputData read_mesh_data(const MeshData& mesh, const MeshData* envelope = nullptr);

/**
 * @brief Read one or multiple meshes and convert them into a tet mesh.
 *
 * Winding number is used to tag inside/outside.
 */
InputData read_mesh(
    const std::vector<std::string>& input_paths,
    const std::string& output_filename,
    const nlohmann::json& json_params,
    const io::ComponentIO* component_io = nullptr);

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
    const io::ComponentIO* component_io = nullptr);

/**
 * @brief Decide whether an input is 2D or 3D.
 *
 * `dimension` is "auto" (the default), 2 or 3. Auto looks at the first input:
 *   - .msh                            -> 0, meaning "the msh reader decides from the content"
 *   - OBJ/OFF/STL with any 'f' record -> 3, a surface for EmbedSurface
 *   - OBJ with 'l' but no 'f'         -> 2, a curve network for EmbedCurves
 *
 * An explicit 2 or 3 forces the route; it does not skip the sniff, it disagrees with it
 * loudly, because the failure otherwise surfaces as a confusing parse error much later.
 */
int resolve_input_dimension(
    const std::vector<std::string>& input_paths,
    const nlohmann::json& json_params,
    const io::ComponentIO* component_io = nullptr);

} // namespace wmtk::components::simwild