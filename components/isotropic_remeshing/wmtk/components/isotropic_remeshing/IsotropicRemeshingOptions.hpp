#pragma once
#include <memory>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/components/output/OutputOptions.hpp>
#include "EdgeSwapMode.hpp"


namespace wmtk::components::multimesh {
class MeshCollection;
}
namespace wmtk::components::output {
class MeshCollection;
}

namespace wmtk::components::isotropic_remeshing {


struct IsotropicRemeshingOptions
{
    // std::shared_ptr<TriMesh> input;
    wmtk::attribute::MeshAttributeHandle position_attribute;
    std::optional<wmtk::attribute::MeshAttributeHandle> inversion_position_attribute;
    std::vector<wmtk::attribute::MeshAttributeHandle> other_position_attributes;
    std::optional<wmtk::attribute::MeshAttributeHandle> sizing_field_attribute;
    std::optional<wmtk::attribute::MeshAttributeHandle> visited_edge_flag;
    std::optional<wmtk::attribute::MeshAttributeHandle> target_edge_length;

    // tag attributes can be split into two, or two of the same tag can merge into one
    std::vector<wmtk::attribute::MeshAttributeHandle> tag_attributes;

    std::vector<wmtk::attribute::MeshAttributeHandle> pass_through_attributes;
    int64_t iterations = 10;
    double length_abs = 0;
    double length_rel = 0;
    bool lock_boundary = true;
    bool use_for_periodic = false;
    bool fix_uv_seam = true;

    bool use_split = true;
    bool use_swap = true;
    bool use_collapse = true;
    bool use_smooth = true;
    // this should be true for periodic
    bool separate_substructures = false;

    EdgeSwapMode edge_swap_mode = EdgeSwapMode::Skip;

    std::optional<double> envelope_size; // 1e-3


    void load_json(const nlohmann::json& js);
    void write_json(nlohmann::json& js) const;


    // transforms the absoltue or relative length paramters into an absolute length parameter
    double get_absolute_length() const;


    friend void to_json(
        nlohmann::json& nlohmann_json_j,
        const IsotropicRemeshingOptions& nlohmann_json_t);
    friend void from_json(
        const nlohmann::json& nlohmann_json_j,
        IsotropicRemeshingOptions& nlohmann_json_t);

    std::vector<wmtk::attribute::MeshAttributeHandle> all_positions() const;


    wmtk::components::multimesh::MeshCollection* mesh_collection;
    // format for outputting intermediate results. Assumed to just be a frame number, i.e something
    // like format("path_{}.hdf5",0) to generate path_0.hdf5
    std::map<std::string, wmtk::components::output::OutputOptions> intermediate_output_format;
};


} // namespace wmtk::components::isotropic_remeshing
