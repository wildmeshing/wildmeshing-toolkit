#pragma once

#include <igl/Timer.h>
#include "Parameters.h"
#include <wmtk/TriMesh.h>


namespace triwild {

class VertexAttributes
{
public:
    Eigen::Vector2d pos; 

    size_t partition_id = 0; // TODO this should not be here

};


class FaceAttributes
{
public:
};

class TriWild : public wmtk::TriMesh
{
public:

TriWild() {};

virtual ~TriWild() {};


// Store the per-vertex attributes
wmtk::AttributeCollection<VertexAttributes> vertex_attrs;

// Initializes the mesh
void create_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

// Writes a triangle mesh in OBJ format
bool write_mesh(std::string path);

// Smoothing
void smooth_all_vertices();
bool smooth_before(const Tuple& t) override;
bool smooth_after(const Tuple& t) override;


};

} // namespace tetwild
