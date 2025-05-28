#pragma once
#include <wmtk/Mesh.hpp>

namespace wmtk::components {

struct EnvelopeOptions
{
    /**
     * Name of the envelope. Mostly used for debugging.
     */
    std::string envelope_name;
    /**
     * The mesh that is used to construct the envelope.
     */
    std::shared_ptr<Mesh> envelope_constrained_mesh;
    /**
     * The mesh that is checked if it is within the envelope.
     */
    std::shared_ptr<Mesh> envelope_geometry_mesh;
    /**
     * The position attribute name of the envelope-construction mesh.
     */
    std::string constrained_position_name = "vertices";
    /**
     * The position attribute name for the mesh that must remain within the envelope.
     */
    std::string geometry_position_name = "vertices";
    /**
     * The envelope thickness, relative to the bounding box of the embedding.
     */
    double thickness = 1e-6;
};


struct WildMeshingOptions
{
    /**
     * The mesh that is embedding the submeshes (represented by the envelope options).
     */
    std::shared_ptr<Mesh> input_mesh;
    /**
     * The position attribute of the input mesh, i.e., the embedding.
     */
    std::string input_mesh_position = "vertices";
    /**
     * Target edge length, relative to the bounding box of the input mesh.
     */
    double target_edge_length = 0.05;
    /**
     * The targeted max amips. The optimization will continue until every cell has an amips below
     * the max or until it runs out of iterations.
     */
    double target_max_amips = 10.0;
    /**
     * The maximum number of iterations of the optimization.
     */
    int64_t max_passes = 10;
    /**
     * Debugging output.
     */
    bool intermediate_output = false;
    /**
     * Replace the `double` vertex positions by `Rational` positions. Use this if your mesh has its
     * positions stored as `double`.
     */
    bool replace_double_coordinate = false;
    /**
     * TODO document (I don't know what this is good for)
     */
    size_t scheduler_update_frequency = 0;
    std::string intermediate_output_path = "";
    std::string intermediate_output_name;

    bool skip_split = false;
    bool skip_collapse = false;
    bool skip_swap = false;
    bool skip_smooth = false;

    /**
     * The envelopes ensure that substructures remain in their position up to a given error.
     */
    std::vector<EnvelopeOptions> envelopes;
    /**
     * Further attributes that will be handled with default behavior.
     */
    std::vector<attribute::MeshAttributeHandle> pass_through;

    /**
     * Convert the multimesh into tags und use Embedding/SubMesh.
     * For now, no topologica checks on the submesh will be performed.
     */
    bool use_embedding = false;
};

} // namespace wmtk::components