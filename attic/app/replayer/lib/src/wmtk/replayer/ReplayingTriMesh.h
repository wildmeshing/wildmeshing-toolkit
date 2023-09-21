#pragma once

#include <wmtk/AttributeCollectionReplayer.h>
#include <wmtk/TriMesh.h>


namespace wmtk {

class ReplayingTriMesh : public TriMesh
{
public:
    ReplayingTriMesh(HighFive::File& record_file);
    ReplayingTriMesh(
        HighFive::File& record_file,
        std::unique_ptr<AttributeCollectionReplayer>&& vertex_attr,
        std::unique_ptr<AttributeCollectionReplayer>&& vertex_attr,
        std::unique_ptr<AttributeCollectionReplayer>&& vertex_attr);


    // after initialize is called it is unsafe to change the replayer objects
    void update_size();

    void step_forward();
    void step_backward();

private:
    ReplayingTriMesh(HighFive::File& file);
    AttributeCollectionReplayer face_replayer;
    std::unique_ptr<AttributeCollectionReplayer> vertex_attr_replayer = nullptr;
    std::unique_ptr<AttributeCollectionReplayer> edge_attr_replayer = nullptr;
    std::unique_ptr<AttributeCollectionReplayer> face_attr_replayer = nullptr;
    size_t operation_count = 0;
};

template <typename A, typename B, typename C>
ReplayingTriMesh::ReplayingTriMesh(
    HighFive::File& record_file,
    std::pair < AttributeCollection<A>&,
    AttributeCollection<A>& a,
    AttributeCollection<A>& b,
    AttributeCollection<A>& c)
    : record_file
{}
} // namespace wmtk
