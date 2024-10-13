#include <nlohmann/json.hpp>
#include "IsotropicRemeshingOptions.hpp"
#include <algorithm>


namespace wmtk::components::isotropic_remeshing {
    namespace {

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    IsotropicRemeshingOptions,
    iterations ,
    length_abs ,
    length_rel ,
    lock_boundary ,
    use_for_periodic ,
    dont_disable_split ,
    fix_uv_seam )
    }
    void IsotropicRemeshingOptions::load_json(const nlohmann::json& js) {

        from_json(js,*this);
    }
    void IsotropicRemeshingOptions::write_json(nlohmann::json& js) const {
        to_json(js,*this);
    }

    std::vector<wmtk::attribute::MeshAttributeHandle> IsotropicRemeshingOptions::all_positions() const {
        auto r = other_position_attributes;
        r.emplace_back(position_attribute);
        if(inversion_position_attribute.has_value()) {
            r.emplace_back(inversion_position_attribute.value());
        }
        std::sort(r.begin(),r.end());

        r.erase(std::unique(r.begin(),r.end()),r.end());

        return r;
    }
}
