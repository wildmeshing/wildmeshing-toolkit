#pragma once

#include <cmath>
#include <memory>

namespace wmtk::operations {
    template <typename T,typename U>
        class SingleAttributeTransferStrategy;
}
namespace wmtk::components::isotropic_remeshing::internal {

        std::shared_ptr<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            amips_handle,
            position_handle,
            compute_amips);
}
