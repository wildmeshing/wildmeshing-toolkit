#include "Manifold-extraction.hpp"

namespace wmtk {
    auto pntgen2d(const size_t nb_points, const double range) -> std::vector<wmtk::Point2D>{
        std::vector<Point2D> points(nb_points);
        std::random_device rd{};
        std::mt19937 gen(rd()); //std::mt19937 gen(10);
        std::uniform_real_distribution<double> dis(0.0, range);
        for (size_t i = 0; i < nb_points; ++i) {
            // generate 2 random doubles between 0 and the given range
            for (auto j = 0; j < 2; j++) {
                points[i][j] = dis(gen);  //(std::rand() % 10 + 1);
            }
        }
        return points;
    }

    auto pntgen3d(size_t nb_points, double range) -> std::vector<wmtk::Point3D>{
        std::vector<Point3D> points(nb_points);
        std::random_device rd{};
        std::mt19937 gen(rd()); //std::mt19937 gen(10);
        std::uniform_real_distribution<double> dis(0.0, range);
        for (size_t i = 0; i < nb_points; ++i) {
            // generate 3 random doubles between 0 and the given range
            for (auto j = 0; j < 3; j++) {
                points[i][j] = dis(gen);  //(std::rand() % 10 + 1);
            }
        }
        return points;
    }

    auto tagassign(size_t nb_triangles) -> std::vector<size_t>{
        std::vector<size_t> tagass;
        std::srand(10);
        for (size_t i = 0 ; i < nb_triangles; i++){
            size_t tag = std::rand() % 2;
            if (tag == 1) tagass.push_back(i); // for every tagged triangle, add its index to the end
        }
        return tagass;
    }




    // // Function to find connected components in a set of triangles
    // std::vector<std::vector<Triangle>> findConnectedComponents(std::vector<Triangle> triangles, std::vector<size_t> index) {
    //     int n = triangles.size();
    //     std::map<size_t, size_t> tagass;
    //     for (int i = 0 ;i < tag.size(); i++){
    //         tagass.insert({i, index[i]});
    //     }
        
    //     std::vector<bool> visited(n, false);
    //     std::vector<std::vector<Triangle>> components;

    //     for (int i = 0; i < n; ++i) {

    //         if (!visited[i]) {
    //             std::vector<int> component;
    //             dfs(triangles, i, visited, component);
    //             std::vector<Triangle> componentTriangles;
    //             for (int idx : component) {
    //                 componentTriangles.push_back(triangles[idx]);
    //             }
    //             components.push_back(componentTriangles);
    //         }
    //     }

    //     return components;
    // }

// // Helper function to check if two triangles share an edge
// bool shareEdge(const Triangle& t1, const Triangle& t2) {
//     std::unordered_set<Point> points1 = {t1.a, t1.b, t1.c};
//     int sharedPoints = 0;
//     if (points1.count(t2.a)) sharedPoints++;
//     if (points1.count(t2.b)) sharedPoints++;
//     if (points1.count(t2.c)) sharedPoints++;
//     return sharedPoints == 2; // Two shared points indicate a shared edge
// }

// // Depth-first search to find connected components
// void dfs(const std::vector<Triangle>& triangles, int current, std::vector<bool>& visited, std::vector<int>& component) {
//     visited[current] = true;
//     component.push_back(current);
    
//     for (int i = 0; i < triangles.size(); ++i) {
//         if (!visited[i] && shareEdge(triangles[current], triangles[i])) {
//             dfs(triangles, i, visited, component);
//         }
//     }

}
