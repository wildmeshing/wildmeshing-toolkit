#include "Reader.hpp"
namespace wmtk {
void stl_to_eigen(std::string input_surface, Eigen::MatrixXd& VI, Eigen::MatrixXi& FI)
{
    GEO::initialize();
    GEO::Mesh input;
    GEO::mesh_load(input_surface, input);
    VI.resize(input.vertices.nb(), 3);
    for (int i = 0; i < VI.rows(); i++)
        VI.row(i) << (input.vertices.point(i))[0], (input.vertices.point(i))[1],
            (input.vertices.point(i))[2];
    input.facets.triangulate();
    // wmtk::logger().info("V {} F {}", input.vertices.nb(), input.facets.nb());
    FI.resize(input.facets.nb(), 3);
    for (int i = 0; i < FI.rows(); i++)
        FI.row(i) << input.facets.vertex(i, 0), input.facets.vertex(i, 1),
            input.facets.vertex(i, 2);
}
void eigen_to_wmtk_input(
    std::vector<Eigen::Vector3d>& verts,
    std::vector<std::array<size_t, 3>>& tris,
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F)
{
    for (int i = 0; i < V.rows(); i++) {
        verts[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tris[i][j] = (size_t)F(i, j);
    }
}

void resolve_duplicated_faces(const Eigen::MatrixXi& inF, Eigen::MatrixXi& outF)
{
    std::map<std::array<int, 3>, int> unique;
    std::vector<Eigen::Vector3i> newF;
    newF.reserve(inF.rows());
    for (auto i = 0; i < inF.rows(); i++) {
        std::array<int, 3> tri;
        for (auto j = 0; j < 3; j++) tri[j] = inF(i, j);

        std::sort(tri.begin(), tri.end());
        auto [it, suc] = unique.emplace(tri, i);
        if (suc) {
            newF.emplace_back(inF.row(i));
        }
    }
    outF.resize(newF.size(), 3);
    for (auto i = 0; i < newF.size(); i++) {
        outF.row(i) = newF[i];
    }
}

void stl_to_manifold_wmtk_input(
    std::string input_path,
    double remove_duplicate_esp,
    std::pair<Eigen::Vector3d, Eigen::Vector3d>& box_minmax,
    std::vector<Eigen::Vector3d>& verts,
    std::vector<std::array<size_t, 3>>& tris,
    std::vector<size_t>& modified_nonmanifold_v)
{
    Eigen::MatrixXd inV, V;
    Eigen::MatrixXi inF, F;
    wmtk::stl_to_eigen(input_path, inV, inF);
    Eigen::VectorXi _I;

    igl::remove_unreferenced(inV, inF, V, F, _I);

    if (V.rows() == 0 || F.rows() == 0) {
        wmtk::logger().info("== finish with Empty Input, stop.");
        exit(0);
    }

    box_minmax = std::pair(V.colwise().minCoeff(), V.colwise().maxCoeff());
    double diag = (box_minmax.first - box_minmax.second).norm();

    // using the same error tolerance as in tetwild
    Eigen::VectorXi SVI, SVJ, SVK;
    Eigen::MatrixXd temp_V = V; // for STL file
    igl::remove_duplicate_vertices(
        temp_V,
        std::min(1e-5, remove_duplicate_esp / 10 * diag),
        V,
        SVI,
        SVJ);
    for (int i = 0; i < F.rows(); i++)
        for (int j : {0, 1, 2}) F(i, j) = SVJ[F(i, j)];
    auto F1 = F;

    resolve_duplicated_faces(F1, F);


    verts.resize(V.rows());
    tris.resize(F.rows());
    wmtk::eigen_to_wmtk_input(verts, tris, V, F);

    wmtk::logger().info("after remove duplicate v#: {} f#: {}", V.rows(), F.rows());


    Eigen::VectorXi dummy;

    if (!igl::is_edge_manifold(F) || !igl::is_vertex_manifold(F, dummy)) {
        auto v1 = verts;
        auto tri1 = tris;
        wmtk::separate_to_manifold(v1, tri1, verts, tris, modified_nonmanifold_v);
    }
}
} // namespace wmtk