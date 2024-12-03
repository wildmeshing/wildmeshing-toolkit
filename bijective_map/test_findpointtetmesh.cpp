#include <iostream>
#include "FindPointTetMesh.hpp"

int main()
{
    // Example data
    Eigen::MatrixXd V(5, 3); // Vertex matrix (5 vertices)
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1;

    Eigen::MatrixXi T(1, 4); // Tetrahedron matrix (1 tetrahedron)
    T << 0, 1, 2, 3;

    Eigen::Vector3d p(0.2, 0.2, 0.2); // Query point

    // Call the function
    auto result = findTetContainingPoint(V, T, p);

    if (result.first != -1) {
        std::cout << "Point is in tetrahedron: " << result.first << std::endl;
        std::cout << "Barycentric coordinates: " << result.second.transpose() << std::endl;
    } else {
        std::cout << "Point is not inside any tetrahedron!" << std::endl;
    }

    return 0;
}