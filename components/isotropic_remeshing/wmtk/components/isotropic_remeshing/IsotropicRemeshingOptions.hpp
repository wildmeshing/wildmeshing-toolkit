#pragma once
#include <memory>
#include <wmtk/attribute/MeshAttributeHandle.hpp>


namespace wmtk::components::isotropic_remeshing {

struct IsotropicRemeshingAttributes
{
    wmtk::attribute::MeshAttributeHandle position;
    std::optional<wmtk::attribute::MeshAttributeHandle> inversion_position;
    std::vector<wmtk::attribute::MeshAttributeHandle> other_positions;
    bool update_other_positions;
};

struct IsotropicRemeshingOptions
{
    // std::shared_ptr<TriMesh> input;
    IsotropicRemeshingAttributes attributes;
    std::vector<wmtk::attribute::MeshAttributeHandle> pass_through;
    int64_t iterations = 10;
    double length_abs = 0;
    double length_rel = 0;
    bool lock_boundary = true;
    bool use_for_periodic = false;
    bool dont_disable_split = false;
    bool fix_uv_seam = true;
};


} // namespace wmtk::components::isotropic_remeshing
