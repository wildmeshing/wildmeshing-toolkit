#pragma once

#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>

namespace wmtk::operations::composite {

/**
 * A FaceSwap (or a Swap 2-3) performs a swap a face sharing by two tets to an edge split the volume
 * of the two tets into three new tets.
 *
 * This is the reverse operation of a EdgeSwap 3-2.
 *
 * FaceSwap executes two splits then two collapses.
 *
 * FaceSwap cannot be performed on boundary faces.
 *
 *  *
 *        0 ---------- 4
 *       / \\        /
 *      /   \ \     /
 *     /     \  \  /
 *    /       \  \\ 3
 *  1 --------- 2/         input: (0, 0-2, 0-2-3, 0-2-3-1)
 *
 *
 *        0 ---------- 4
 *       / \\     _- /
 *      /   \ \ _-  /
 *     /   __\- \  /
 *    /_--    \  \\ 3
 *  1 --------- 2/         output: (4, 4-1, 4-1-3, 4-1-3-2)
 *
 *
 */

class TetFaceSwap : public Operation
{
public:
    TetFaceSwap(Mesh& m);

    PrimitiveType primitive_type() const override { return PrimitiveType::Triangle; }

    inline EdgeSplit& split() { return m_split; }
    inline EdgeCollapse& collapse() { return m_collapse; }

protected:
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

private:
    EdgeSplit m_split;
    EdgeCollapse m_collapse;
};

} // namespace wmtk::operations::composite