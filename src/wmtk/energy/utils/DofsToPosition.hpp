#pragma once
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
    wmtk::image::SamplingBicubic m_sampling;

public:
    DofsToPosition() = default;
    ~DofsToPosition() = default;
    DofsToPosition& operator=(const DofsToPosition&) = default; // copy assignment operator
    DofsToPosition& operator=(DofsToPosition&&) = default; // move assignment operator
    DofsToPosition(const image::Image& image)
        : m_sampling(image)
    {}

    template <typename T>
    Eigen::Matrix<T, 3, 1> dof_to_pos(const Eigen::Matrix<double, Eigen::Dynamic, 1>& dof) const
    {
        Eigen::Matrix<T, 3, 1> pos;
        int size = dof.rows();

        if (size == 2) {
            Eigen::Matrix<T, 2, 1> dofT;
            get_local_vector<Eigen::Matrix<T, 2, 1>>(dof, size, dofT);
            // TODO retrive position using displacement map
            // for now just return itself
            pos << dofT(0), dofT(1), m_sampling.sample_T(dofT(0), dofT(1));

        } else
        //(dofx.size() == 1)
        {
            Eigen::Matrix<T, 1, 1> dofT;
            get_local_vector<Eigen::Matrix<T, 1, 1>>(dof, size, dofT);
            // curve parameterization
            // TODO can also be implemented as a overload operator()?

            pos << dofT(0), static_cast<T>(0.0), static_cast<T>(0.0);
        }
        return pos;
    }
};

} // namespace energy
} // namespace wmtk
