#include <wmtk/TriMesh.hpp>
#include <wmtk/energy/utils/AutoDiffTypes.hpp>
#include "autodiff.h"
// #include <wmtk/image/DisplacementMap.hpp>

namespace wmtk {
namespace energy {
class DofsToPosition
{ // size = 2: uv position, size =1, t of boundary curve
    using DofVectorX = Eigen::Matrix<double, Eigen::Dynamic, 1>;

public:
    DofsToPosition();
    ~DofsToPosition();

    template <typename T>
    Eigen::Matrix<T, 3, 1> operator()(const DofVectorX& dof) const
    {
        Eigen::Matrix<T, 3, 1> pos;
        int size = dof.rows();
        typedef Eigen::Matrix<T, size, 1> Vec2T;
        Vec2T dofT;
        get_local_vector<Vec2T>(dof, size, dofT);
        if (size == 2) {
            // TODO retrive position using displacement map
            // for now just return itself
            pos << dofT(0), dofT(1), static_cast<T>(0.0);
        } else
        //(dofx.size() == 1)
        {
            // curve parameterization
            // TODO can also be implemented as a overload operator()?

            pos << dofT(0), static_cast<T>(0.0), static_cast<T>(0.0);
        }
        return pos;
    }
}

} // namespace energy
} // namespace wmtk
