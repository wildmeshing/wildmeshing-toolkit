#include <nlohmann/json.hpp>
#include "IsotropicRemeshingOptions.hpp"


namespace wmtk::components::isotropic_remeshing {
    namespace {

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    IsotropicRemeshingOptions,
    update_other_positions,
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
}
