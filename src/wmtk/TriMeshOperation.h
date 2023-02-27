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
    struct ExecuteReturnData
    {
        Tuple tuple;
        std::vector<Tuple> new_tris;
        bool success = false;
        operator bool() const { return success;}
    };

    ExecuteReturnData operator()(TriMesh& m, const Tuple& t);
    virtual std::string name() const = 0;


    TriMeshOperation() {}
    virtual ~TriMeshOperation() {}

protected:
    // returns the changed tris + whether success occured
    virtual ExecuteReturnData execute(TriMesh& m, const Tuple& t) = 0;
    virtual bool before(TriMesh& m, const Tuple& t) = 0;
    virtual bool after(TriMesh& m, ExecuteReturnData& ret_data) = 0;
    virtual bool invariants(TriMesh& m, ExecuteReturnData& ret_data);


    // forwarding of operations in TriMesh
    static wmtk::AttributeCollection<VertexConnectivity>& vertex_connectivity(TriMesh& m);
    static wmtk::AttributeCollection<TriangleConnectivity>& tri_connectivity(TriMesh& m);
    static const wmtk::AttributeCollection<VertexConnectivity>& vertex_connectivity(
        const TriMesh& m);
    static const wmtk::AttributeCollection<TriangleConnectivity>& tri_connectivity(
        const TriMesh& m);

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


template <
    typename MeshType,
    typename DerivedOperationType,
    typename BaseOperationType = TriMeshOperation
#if defined(__cpp_concepts)
    >
    requires(
        std::is_base_of_v<TriMesh, MeshType> &&
        std::is_base_of_v<TriMeshOperation, BaseOperationType>)
#else
    ,
    typename = std::enable_if_t<std::is_base_of_v<TriMesh, MeshType>, void>,
    typename = std::enable_if_t<std::is_base_of_v<TriMeshOperation, BaseOperationType>, void>>
#endif
class TriMeshOperationShim : public BaseOperationType
{
public:
    using ExecuteReturnData = TriMeshOperation::ExecuteReturnData;
    using Tuple = TriMeshOperation::Tuple;
    DerivedOperationType& derived() { return static_cast<DerivedOperationType&>(*this); }
    const DerivedOperationType& derived() const
    {
        return static_cast<const DerivedOperationType&>(*this);
    }

    ExecuteReturnData execute(TriMesh& m, const Tuple& t) override
    {
        return execute( static_cast<MeshType&>(m), t);
    }
    bool before(TriMesh& m, const Tuple& t) override
    {
        return before(static_cast<MeshType&>(m), t);
    }
    bool after(TriMesh& m, ExecuteReturnData& ret_data) override
    {
        return after( static_cast<MeshType&>(m), ret_data);
    }
    bool invariants(TriMesh& m, ExecuteReturnData& ret_data) override
    {
        return invariants( static_cast<MeshType&>(m), ret_data);
    }

private:
    ExecuteReturnData execute(MeshType& m, const Tuple& t) { return derived().execute(m, t); }
    bool before(MeshType& m, const Tuple& t) { return derived().before(m, t); }
    bool after( MeshType& m, ExecuteReturnData& ret_data)
    {
        return derived().after(m, ret_data);
    }
    bool invariants( MeshType& m, ExecuteReturnData& ret_data)
    {
        return derived().invariants(m, ret_data);
    }
};


class TriMeshSplitEdgeOperation : public TriMeshOperation
{
public:
    ExecuteReturnData execute(TriMesh& m, const Tuple& t) override;
    bool before(TriMesh& m, const Tuple& t) override;
    bool after(TriMesh& m, ExecuteReturnData& ret_data) override;
    std::string name() const override;

    // returns a tuple to the new vertex created by this operation, where the
    // input is the tuple passed into after's ret_data.tuple.
    Tuple new_vertex(TriMesh& m, const Tuple& t) const;
    std::array<Tuple, 2> original_endpoints(TriMesh& m, const Tuple& t) const;
};
class TriMeshSwapEdgeOperation : public TriMeshOperation
{
public:
    ExecuteReturnData execute(TriMesh& m, const Tuple& t) override;
    bool before(TriMesh& m, const Tuple& t) override;
    bool after(TriMesh& m, ExecuteReturnData& ret_data) override;
    std::string name() const override;
};

class TriMeshEdgeCollapseOperation : public TriMeshOperation
{
public:
    ExecuteReturnData execute(TriMesh& m, const Tuple& t) override;
    bool before(TriMesh& m, const Tuple& t) override;
    bool after(TriMesh& m, ExecuteReturnData& ret_data) override;
    std::string name() const override;

    static bool check_link_condition(const TriMesh& m, const Tuple& t);
};

class TriMeshSmoothVertexOperation : public TriMeshOperation
{
public:
    ExecuteReturnData execute(TriMesh& m, const Tuple& t) override;
    bool before(TriMesh& m, const Tuple& t) override;
    bool after(TriMesh& m, ExecuteReturnData& ret_data) override;
    std::string name() const override;
    bool invariants(TriMesh& m, ExecuteReturnData& ret_data) override;
};

class TriMeshConsolidateOperation : public TriMeshOperation
{
public:
    ExecuteReturnData execute(TriMesh& m, const Tuple& t) override;
    bool before(TriMesh& m, const Tuple& t) override;
    bool after(TriMesh& m, ExecuteReturnData& ret_data) override;
    std::string name() const override;
};
} // namespace wmtk
