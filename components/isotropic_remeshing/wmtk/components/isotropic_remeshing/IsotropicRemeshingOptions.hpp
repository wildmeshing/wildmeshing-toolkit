#pragma once
#include <memory>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include "EdgeSwapMode.hpp"


namespace wmtk::components::isotropic_remeshing {


struct IsotropicRemeshingOptions
{
    // std::shared_ptr<TriMesh> input;
    wmtk::attribute::MeshAttributeHandle position_attribute;
    std::optional<wmtk::attribute::MeshAttributeHandle> inversion_position_attribute;
    std::vector<wmtk::attribute::MeshAttributeHandle> other_position_attributes;

    std::vector<wmtk::attribute::MeshAttributeHandle> pass_through_attributes;
    int64_t iterations = 10;
    double length_abs = 0;
    double length_rel = 0;
    bool lock_boundary = true;
    bool use_for_periodic = false;
    bool fix_uv_seam = true;

    EdgeSwapMode edge_swap_mode = EdgeSwapMode::Skip;

    std::optional<double> envelope_size; // 1e-3


    void load_json(const nlohmann::json& js);
    void write_json(nlohmann::json& js) const;


    // transforms the absoltue or relative length paramters into an absolute length parameter
    double get_absolute_length() const;



    friend void to_json(nlohmann::json& nlohmann_json_j, const IsotropicRemeshingOptions& nlohmann_json_t);
    friend void from_json(
        const nlohmann::json& nlohmann_json_j,
        IsotropicRemeshingOptions& nlohmann_json_t);

    std::vector<wmtk::attribute::MeshAttributeHandle> all_positions() const;
};


} // namespace wmtk::components::isotropic_remeshing
