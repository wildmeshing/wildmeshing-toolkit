#include "isotropic_remeshing.hpp"

#include "IsotropicRemeshing.hpp"

namespace wmtk::components::isotropic_remeshing {


void isotropic_remeshing(const IsotropicRemeshingOptions& options)
{
    IsotropicRemeshing app(options);
    app.run();
}
} // namespace wmtk::components::isotropic_remeshing
