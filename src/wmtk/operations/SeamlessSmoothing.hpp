#pragma once

#include <polysolve/Types.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/function/Function.hpp>
#include "AttributesUpdate.hpp"
#include "OptimizationSmoothing.hpp"

namespace polysolve::nonlinear {
class Solver;
}

namespace wmtk::function {
class Function;
}

namespace wmtk::operations {

class SeamlessSmoothing : public OptimizationSmoothing
{
    // private:
    // class WMTKProblem;

public:
    SeamlessSmoothing(
        TriMesh& ref_mesh,
        TriMesh& cut_mesh,
        std::shared_ptr<wmtk::function::Function> energy);

    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

private:
    TriMesh& m_cut_mesh;
    TriMesh& m_ref_mesh;
};

} // namespace wmtk::operations
