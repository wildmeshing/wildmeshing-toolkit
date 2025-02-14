#pragma once
#include <wmtk/submesh/Embedding.hpp>
#include <wmtk/submesh/SubMesh.hpp>

namespace wmtk::components {

struct EmbeddingEnvelopeOptions
{
    std::string envelope_name;
    std::shared_ptr<submesh::SubMesh> envelope_constrained_mesh;
    std::shared_ptr<submesh::SubMesh> envelope_geometry_mesh;
    std::string constrained_position_name;
    std::string geometry_position_name;
    double thickness;
};


struct WildMeshingEmbeddingOptions
{
    std::shared_ptr<submesh::Embedding> input_mesh;
    std::string input_mesh_position;
    double target_edge_length;
    double target_max_amips;
    double max_passes;
    bool intermediate_output;
    bool replace_double_coordinate;
    size_t scheduler_update_frequency;
    std::string intermediate_output_path;
    std::string intermediate_output_name;

    bool skip_split = false;
    bool skip_collapse = false;
    bool skip_swap = false;
    bool skip_smooth = false;

    std::vector<EmbeddingEnvelopeOptions> envelopes;
    std::vector<attribute::MeshAttributeHandle> pass_through;
};

} // namespace wmtk::components