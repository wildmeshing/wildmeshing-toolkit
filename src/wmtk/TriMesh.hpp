
class TriMesh : public Mesh
{
private:
    MeshAttributeHandle<long> m_vf_handle;
    MeshAttributeHandle<long> m_ef_handle;

    MeshAttributeHandle<long> m_fv_handle;
    MeshAttributeHandle<long> m_fe_handle;
    MeshAttributeHandle<long> m_ff_handle;
    std::vector<Tuple> get_vertices() const; 
    std::vector<Tuple> get_edges() const ;
    std::vector<Tuple> get_faces() const ;

public:
    TriMesh();

    void split_edge(const Tuple& t) override;
    void collapse_edge(const Tuple& t) override;
    long id(const Tuple& tuple, const PrimitiveType& type) const override;
    Tuple switch_tuple(const Tuple& tuple, const PrimitiveType& type) const override;
    bool is_ccw(const Tuple& tuple) const override;
    void initialize(
        Eigen::Ref<const RowVectors3l>& FV,
        Eigen::Ref<const RowVectors3l>& FE,
        Eigen::Ref<const RowVectors3l>& FF,
        Eigen::Ref<const VectorXl>& VF,
        Eigen::Ref<const VectorXl>& EF,
        Eigen::Ref<const RowVectors4l>& seam) const;

};

// TODO: this trimesh_topology_initialization should belong in a detail folder or something, not part of the trimesh class
/**
 * @brief given the mesh connectivity in matrix format, initialize the topology data used for Mesh
 * @param F input connectivity in (N x 3) matrix format (igl convention)
 * @param FV output connectivity in (N x 3) matrix format, same as F
 * @param FE three edges of every triangle in (N x 3) matrix format
 * @param FF three edge-adjacent faces of every triangle in (N x 3) matrix format
 * @param VF one adjacent triangle (arbitrarily chosen) of every vertex in (N x 1) matrix format
 * @param EF one adjacent triangle (arbitrarily chosen) of every edge in (N x 1) matrix format
 */
void trimesh_topology_initialization(
    Eigen::Ref<const Mesh::RowVectors3l> F,
    Eigen::Ref<Mesh::RowVectors3l> FV,
    Eigen::Ref<Mesh::RowVectors3l> FE,
    Eigen::Ref<Mesh::RowVectors3l> FF,
    Eigen::Ref<Mesh::VectorXl> VF,
    Eigen::Ref<Mesh::VectorXl> EF);
