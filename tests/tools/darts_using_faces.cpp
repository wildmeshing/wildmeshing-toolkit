#include "darts_using_faces.hpp"
#include <wmtk/autogen/SimplexDart.hpp>


namespace wmtk::tests::tools {

    std::vector<autogen::Dart> darts_using_faces(PrimitiveType mesh_type, 
            const std::array<std::optional<int64_t>,3>& indices) {


        const auto& sd = wmtk::autogen::SimplexDart::get_singleton(mesh_type);


    }

}
