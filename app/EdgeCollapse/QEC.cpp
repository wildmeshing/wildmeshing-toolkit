#include <wmtk/TriMesh.h>
#include <wmtk/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
void add_Qs(std::array<double, 10> Q1, std::array<double, 10> Q2)
{
    for (int j = 0; j < 10; j++) {
        Q1[j] = Q1[j] + Q2[j];
    }
}

// get the quadrix in form of an array of 10 floating point numbers
std::array<double, 10> compute_Q_f(wmtk::TriMesh& m, size_t fid)
{
    auto conn_indices = m.m_tri_connectivity[fid].m_indices;
    Eigen::Vector3d A = m_vertices_position[conn_indices[0]];
    Eigen::Vector3d B = m_vertices_position[conn_indices[1]];
    Eigen::Vector3d C = m_vertices_position[conn_indices[2]];

    Eigen::Vector3d n = ((A - B).cross(C - B)).normalize();
    double a = n(0);
    double b = n(1);
    double c = n(2);
    double d = -n.dot(B);

    std::array<double, 10> Q;
    // a^2, ab, ac, ad, b^2, bc, bd, c^2, cd, d^2,
    Q[0] = a * a;
    Q[1] = a * b;
    Q[2] = a * c;
    Q[3] = a * d;
    Q[4] = b * b;
    Q[5] = b * c;
    Q[6] = b * d;
    Q[7] = c * c;
    Q[8] = c * d;
    Q[9] = d * d;
    return Q;
}

std::array<double, 10> compute_Q_v(wmtk::TriMesh& m, size_t vid)
{
    auto conn_tris = m.m_vertex_connectivity[vid].m_conn_tris;
    std::array<double, 10> Q{};
    for (size_t tri : conn_tris) {
        std::array<double, 10> Q_t = compute_Q_f(m, tri, m_vertices_position);
        add_Qs(Q, Q_t);
    }
    return Q;
}

void collapse_queue(wmtk::TriMesh& m)
{
    // find the valid pairs (for each vertex)
    std::vector<Tuple> edges = m.get_edges();
    // find the best pair by keeping the priority queue
    // always keep the high cost one on top of the queue


    // do the edge collapse of the top pair

    // update cost priority queue
}
