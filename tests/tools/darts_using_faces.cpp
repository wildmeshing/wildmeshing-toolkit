#include "darts_using_faces.hpp"
#include <wmtk/autogen/SimplexDart.hpp>
#include <wmtk/autogen/utils/simplex_index_from_valid_index.hpp>


namespace wmtk::tests::tools {

std::vector<int8_t> darts_using_faces(
    PrimitiveType mesh_type,
    const std::array<std::optional<int64_t>, 3>& indices)
{
    const auto& sd = wmtk::autogen::SimplexDart::get_singleton(mesh_type);

    std::vector<int8_t> darts;
    auto matches = [mesh_type](int8_t index) -> bool {
        for (size_t j = 0; j < 3; ++j) {
            const auto& ind_opt = indices[j];
            if (ind_opt.has_value()) {
                const int64_t ind = ind_opt.value();
                const PrimitiveType pt(ind);
                const int8_t got_ind =
                    wmtk::autogen::utils::simplex_index_from_valid_index(mesh_type, index, pt);
                if (ind != got_ind) {
                    return false;
                }
            }
        }
        return true;
    } for (int8_t i = 0; i < sd.size(); ++i)
    {
        if (matches(i)) {
            darts.emplace_back(i);
        }
    }
    return darts;
}

} // namespace wmtk::tests::tools
