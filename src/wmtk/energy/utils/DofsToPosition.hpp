#pragma once
#include <memory>
#include <wmtk/TriMesh.hpp>
#include <wmtk/image/Image.hpp>
#include <wmtk/image/Sampling.hpp>
#include <wmtk/image/bicubic_interpolation.hpp>
#include "AutoDiffUtils.hpp"
#include "autodiff.h"

namespace wmtk {
namespace energy {
class DofsToPosition
{ // size = 2: uv position, size =1, t of boundary curve
    using DofVectorX = Eigen::Matrix<double, Eigen::Dynamic, 1>;

protected:
    std::unique_ptr<wmtk::image::Sampling> m_sampling;

public:
    DofsToPosition() = default;
    ~DofsToPosition() = default;
    DofsToPosition& operator=(const DofsToPosition&) = default; // copy assignment operator
    DofsToPosition& operator=(DofsToPosition&&) = default; // move assignment operator

    /**
     * @brief Construct a new Dofs To Position object using a displacement map (requires a sampler)
     *
     * @param image
     */
    DofsToPosition(const image::Image& image)
    {
        m_sampling = std::make_unique<wmtk::image::SamplingBicubic>(image);
    }

    template <typename T>
    Eigen::Matrix<T, 3, 1> dof_to_pos(const Eigen::Matrix<T, Eigen::Dynamic, 1>& dofT) const
    {
        Eigen::Matrix<T, 3, 1> pos;
        int size = dofT.rows();

        if (size == 2) {
            // TODO retrive position using displacement map
            // for now just return itself
            pos << dofT(0), dofT(1), m_sampling->sample(dofT(0), dofT(1));

        } else
        //(dofx.size() == 1)
        {
            // curve parameterization
            // TODO covert to uv first and sample using the uv

            pos << dofT(0), static_cast<T>(0.0), static_cast<T>(0.0);
        }
        return pos;
    }
};

} // namespace energy
} // namespace wmtk
