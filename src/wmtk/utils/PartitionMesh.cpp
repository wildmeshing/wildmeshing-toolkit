#include <wmtk/utils/PartitionMesh.h>

#include <igl/remove_unreferenced.h>

namespace wmtk {

constexpr auto _partition_faces = [](auto& m, auto num_partition) {
    auto f_tuples = m.get_faces();
    Eigen::MatrixXi F(f_tuples.size(), 3);
    for (int i = 0; i < f_tuples.size(); i++) {
        F(i, 0) = f_tuples[i].vid(m);
        auto e1 = f_tuples[i].switch_vertex(m);
        F(i, 1) = e1.vid(m);
        F(i, 2) = e1.switch_edge(m).switch_vertex(m).vid(m);
    }

    Eigen::VectorXi I, J;
    igl::remove_unreferenced(m.vert_capacity(), F, I, J);
    Eigen::MatrixXi NF = F;
    for (auto i = 0; i < NF.rows(); i++) {
        for (auto j = 0; j < 3; j++) {
            NF(i, j) = I(NF(i, j));
        }
    }

    auto partitioned_v = partition_mesh_vertices(NF, num_partition);
    assert(partitioned_v.size() == J.rows());
    std::vector<size_t> v_pid(m.vert_capacity(), 0);
    for (int i = 0; i < partitioned_v.rows(); i++) {
        v_pid[J[i]] = partitioned_v(i, 0);
    }
    return v_pid;
};

std::vector<size_t> partition_TriMesh(const wmtk::TriMesh& m, int num_partition)
{
    return _partition_faces(m, num_partition);
}

std::vector<size_t> partition_TetMesh(wmtk::TetMesh& m, int num_partition)
{
    return _partition_faces(m, num_partition);
}
} // namespace wmtk