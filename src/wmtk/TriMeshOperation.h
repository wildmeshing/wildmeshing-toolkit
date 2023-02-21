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
    };

    ExecuteReturnData operator()(const Tuple& t, TriMesh& m);
    virtual std::string name() const = 0;


    TriMeshOperation() {}
    virtual ~TriMeshOperation() {}

protected:
    // returns the changed tris + whether success occured
    virtual ExecuteReturnData execute(const Tuple& t, TriMesh& m) = 0;
    virtual bool before_check(const Tuple& t, TriMesh& m) = 0;
    virtual bool after_check(const ExecuteReturnData& ret_data, TriMesh& m) = 0;
    virtual bool invariants(const ExecuteReturnData& ret_data, TriMesh& m);


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

    void set_vertex_size(size_t size, TriMesh& m);
    void set_tri_size(size_t size, TriMesh& m);
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

    ExecuteReturnData execute(const Tuple& t, TriMesh& m) override
    {
        return execute(t, static_cast<MeshType&>(m));
    }
    bool before_check(const Tuple& t, TriMesh& m) override
    {
        return before_check(t, static_cast<MeshType&>(m));
    }
    bool after_check(const ExecuteReturnData& ret_data, TriMesh& m) override
    {
        return after_check(ret_data, static_cast<MeshType&>(m));
    }
    bool invariants(const ExecuteReturnData& ret_data, TriMesh& m) override
    {
        return invariants(ret_data, static_cast<MeshType&>(m));
    }

private:
    ExecuteReturnData execute(const Tuple& t, MeshType& m) { return derived().execute(t, m); }
    bool before_check(const Tuple& t, MeshType& m) { return derived().before_check(t, m); }
    bool after_check(const ExecuteReturnData& ret_data, MeshType& m)
    {
        return derived().after_check(ret_data, m);
    }
    bool invariants(const ExecuteReturnData& ret_data, MeshType& m)
    {
        return derived().invariants(ret_data, m);
    }
};


class TriMeshSplitEdgeOperation : public TriMeshOperation
{
public:
    ExecuteReturnData execute(const Tuple& t, TriMesh& m) override;
    bool before_check(const Tuple& t, TriMesh& m) override;
    bool after_check(const ExecuteReturnData& ret_data, TriMesh& m) override;
    std::string name() const override;

    // returns a tuple to the new vertex created by this operation, where the
    // input is the tuple passed into after_check's ret_data.tuple.
    Tuple new_vertex(const Tuple& t, TriMesh& m) const;
    std::array<Tuple, 2> original_endpoints(const Tuple& t, TriMesh& m) const;
};
class TriMeshSwapEdgeOperation : public TriMeshOperation
{
public:
    ExecuteReturnData execute(const Tuple& t, TriMesh& m) override;
    bool before_check(const Tuple& t, TriMesh& m) override;
    bool after_check(const ExecuteReturnData& ret_data, TriMesh& m) override;
    std::string name() const override;
};

class TriMeshEdgeCollapseOperation : public TriMeshOperation
{
public:
    ExecuteReturnData execute(const Tuple& t, TriMesh& m) override;
    bool before_check(const Tuple& t, TriMesh& m) override;
    bool after_check(const ExecuteReturnData& ret_data, TriMesh& m) override;
    std::string name() const override;

    static bool check_link_condition(const Tuple& t, const TriMesh& m);
};

class TriMeshSmoothVertexOperation : public TriMeshOperation
{
public:
    ExecuteReturnData execute(const Tuple& t, TriMesh& m) override;
    bool before_check(const Tuple& t, TriMesh& m) override;
    bool after_check(const ExecuteReturnData& ret_data, TriMesh& m) override;
    std::string name() const override;
    bool invariants(const ExecuteReturnData& ret_data, TriMesh& m) override;
};

class TriMeshConsolidateOperation : public TriMeshOperation
{
public:
    ExecuteReturnData execute(const Tuple& t, TriMesh& m) override;
    bool before_check(const Tuple& t, TriMesh& m) override;
    bool after_check(const ExecuteReturnData& ret_data, TriMesh& m) override;
    std::string name() const override;
};
} // namespace wmtk
