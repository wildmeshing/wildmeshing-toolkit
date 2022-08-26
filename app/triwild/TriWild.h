#pragma once

#include <igl/Timer.h>
#include "Parameters.h"
#include <wmtk/TriMesh.h>


namespace triwild {

class VertexAttributes
{
public:
    Eigen::Vector3d pos;
};


class FaceAttributes
{
public:
};

class TriWild : public wmtk::TriMesh
{
public:
// Store the per-vertex attributes
wmtk::AttributeCollection<VertexAttributes> vertex_attrs;

// Initializes the mesh
void create_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

// Writes a triangle mesh in OBJ format
bool write_triangle_mesh(std::string path);

};

} // namespace tetwild
