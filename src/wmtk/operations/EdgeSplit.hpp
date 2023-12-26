#pragma once

#include "MeshOperation.hpp"

namespace wmtk::operations {

class EdgeSplit : public MeshOperation
{
public:
    EdgeSplit(Mesh& m);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }

    static std::pair<Tuple, Tuple> new_spine_edges(const Mesh& mesh, const Tuple& new_vertex);
    // std::vector<Tuple> new_edge_tuples();
    // std::vector<Tuple> new_face_tuples();

protected:
    std::vector<Simplex> execute(EdgeMesh& mesh, const Simplex& simplex) override;
    std::vector<Simplex> unmodified_primitives(const EdgeMesh& mesh, const Simplex& simplex)
        const override;

    std::vector<Simplex> execute(TriMesh& mesh, const Simplex& simplex) override;
    std::vector<Simplex> unmodified_primitives(const TriMesh& mesh, const Simplex& simplex)
        const override;

    std::vector<Simplex> execute(TetMesh& mesh, const Simplex& simplex) override;
    std::vector<Simplex> unmodified_primitives(const TetMesh& mesh, const Simplex& simplex)
        const override;

    // private:
    //     std::vector<Tuple> new_edges;
    //     std::vector<Tuple> new_faces;
    // TODISCUSS: since now we can get all the new/deleted simplices, do we want to save all of them in the operation instead of navigate from some tuple?
};

} // namespace wmtk::operations
