#include "isotropic_remeshing.hpp"

#include "IsotropicRemeshing.hpp"

namespace wmtk::components::isotropic_remeshing {


void isotropic_remeshing(const IsotropicRemeshingOptions& options)
{
    assert(options.position_attribute.is_valid());
    IsotropicRemeshing app(options);
    app.run();
}
} // namespace wmtk::components::isotropic_remeshing
