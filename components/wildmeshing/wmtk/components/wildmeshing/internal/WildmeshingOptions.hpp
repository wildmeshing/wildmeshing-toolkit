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
    std::string input_mesh_position;
    double target_edge_length;
    double target_max_amips;
    double max_passes;
    bool intermediate_output;
    bool replace_double_coordinate;
    size_t scheduler_update_frequency;
    std::string intermediate_output_path;
    std::string intermediate_output_name;

    bool skip_split;
    bool skip_collapse;
    bool skip_swap;
    bool skip_smooth;

    std::vector<EnvelopeOptions> envelopes;
    std::vector<attribute::MeshAttributeHandle> pass_through;
};

} // namespace wmtk::components