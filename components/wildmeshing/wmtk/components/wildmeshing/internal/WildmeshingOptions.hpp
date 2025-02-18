#pragma once
#include <wmtk/Mesh.hpp>

namespace wmtk::components {

struct EnvelopeOptions
{
    std::string envelope_name;
    std::shared_ptr<Mesh> envelope_constrained_mesh;
    std::shared_ptr<Mesh> envelope_geometry_mesh;
    std::string constrained_position_name;
    std::string geometry_position_name;
    double thickness;
};


struct WildMeshingOptions
{
    std::shared_ptr<Mesh> input_mesh;
    std::string input_mesh_position = "vertices";
    double target_edge_length = 0.05;
    double target_max_amips = 10.0;
    double max_passes = 10;
    bool intermediate_output = false;
    bool replace_double_coordinate = false;
    size_t scheduler_update_frequency = 0;
    std::string intermediate_output_path = "";
    std::string intermediate_output_name;

    bool skip_split = false;
    bool skip_collapse = false;
    bool skip_swap = false;
    bool skip_smooth = false;

    std::vector<EnvelopeOptions> envelopes;
    std::vector<attribute::MeshAttributeHandle> pass_through;

    /**
     * Convert the multimesh into tags und use Embedding/SubMesh.
     * For now, no topologica checks on the submesh will be performed.
     */
    bool use_embedding = false;
};

} // namespace wmtk::components