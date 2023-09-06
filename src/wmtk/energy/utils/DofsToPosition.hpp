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
template <typename SamplingType>
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

    DofsToPosition(std::function<SamplingType(SamplingType, SamplingType)> f)
    {
        m_sampling = std::make_unique<wmtk::image::SamplingAnalyticFunction<SamplingType>>(f);
    }

    /**
     * @brief Construct a new Dofs To Position object using a displacement map (requires a sampler)
     *
     * @param image
     */
    DofsToPosition(const image::Image& image)
    {
        m_sampling = std::make_unique<wmtk::image::SamplingBicubic>(image);
    }


    Eigen::Matrix<SamplingType, 3, 1> dof_to_pos(
        const Eigen::Matrix<double, Eigen::Dynamic, 1>& dof) const
    {
        Eigen::Matrix<SamplingType, 3, 1> pos;
        int size = dof.rows();

        if (size == 2) {
            Eigen::Matrix<SamplingType, 2, 1> dofT;
            get_T_vector<Eigen::Matrix<SamplingType, 2, 1>>(dof, size, dofT);
            // TODO retrive position using displacement map
            // for now just return itself
            pos << dofT(0), dofT(1), m_sampling->sample(dofT(0), dofT(1));

        } else
        //(dofx.size() == 1)
        {
            Eigen::Matrix<SamplingType, 1, 1> dofT;
            get_T_vector<Eigen::Matrix<SamplingType, 1, 1>>(dof, size, dofT);
            // curve parameterization
            // TODO can also be implemented as a overload operator()?

            pos << dofT(0), static_cast<SamplingType>(0.0), static_cast<SamplingType>(0.0);
        }
        return pos;
    }
};

} // namespace energy
} // namespace wmtk
