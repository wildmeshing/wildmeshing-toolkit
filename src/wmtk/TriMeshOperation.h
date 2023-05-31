#pragma once
#include <wmtk/TriMesh.h>
#include <type_traits>

namespace wmtk {
class TriMeshOperation
{
public:
    using Tuple = TriMesh::Tuple;
    using VertexConnectivity = TriMesh::VertexConnectivity;
    using TriangleConnectivity = TriMesh::TriangleConnectivity;

    virtual bool operator()(TriMesh& m, const Tuple& t);
    virtual std::string name() const = 0;


    TriMeshOperation() {}
    virtual ~TriMeshOperation() {}

    virtual double priority(const TriMesh& m, const Tuple& t) const { return 0; }
    virtual std::vector<TriMeshTuple> modified_triangles(const TriMesh& m) const = 0;

protected:
    // returns the changed tris + whether success occured
    virtual bool execute(TriMesh& m, const Tuple& t) = 0;
    virtual bool before(TriMesh& m, const Tuple& t) = 0;
    virtual bool after(TriMesh& m) = 0;
    bool invariants(TriMesh& m);


    // forwarding of operations in TriMesh
    static wmtk::AttributeCollection<VertexConnectivity>& vertex_connectivity(TriMesh& m);
    static wmtk::AttributeCollection<TriangleConnectivity>& tri_connectivity(TriMesh& m);
    static const wmtk::AttributeCollection<VertexConnectivity>& vertex_connectivity(
        const TriMesh& m);
    static const wmtk::AttributeCollection<TriangleConnectivity>& tri_connectivity(
        const TriMesh& m);
    static TriMesh::ProtectedAttributeRAII start_protected_attributes_raii(TriMesh&);
    static TriMesh::ProtectedConnectivityRAII start_protected_connectivity_raii(TriMesh&);
    static void rollback_protected(TriMesh&);

    /**
     * @brief Get the next avaiblie global index for the triangle
     *
     * @return size_t
     */
    size_t get_next_empty_slot_t(TriMesh& m);
    /**
     * @brief Get the next avaiblie global index for the vertex
     *
     * @return size_t
     */
    size_t get_next_empty_slot_v(TriMesh& m);


#if defined(USE_OPERATION_LOGGER)
    std::weak_ptr<OperationRecorder> recorder(TriMesh& m) const;
#endif

    void set_vertex_size(TriMesh& m, size_t size);
    void set_tri_size(TriMesh& m, size_t size);
};



} // namespace wmtk

