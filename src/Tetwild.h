////derived attributes
    class VertexAttr: public BaseVertexAttr { // not part of the mesh class
    public:
        std::vector<Vertex>& m_vertices;
        std::vector<Tet>& m_tets;
        Parameters& m_params;
        Envelope& m_envelope;

        VertexAttr(std::vector<Vertex>& _vertices, std::vector<Tet>& _tets, Parameters& _param, Envelope& _envelope):
                m_vertices(_vertices), m_tets(_tets), m_params(_param), m_envelope(_envelope){}
        ~VertexAttr(){}

        std::vector<Vector3> m_pos;
        std::vector<Vector3f> m_posf;

        std::vector<bool> m_is_on_surface;
        std::vector<bool> m_is_on_boundary;
        std::vector<bool> m_is_on_bbox;
        std::vector<bool> m_is_outside;

        std::vector<Scalar> m_sizing_scalars;
        std::vector<Scalar> m_scalars;
        std::vector<bool> m_is_freezed;

        bool splitting_check_vertex(size_t v1_id, size_t v2_id, size_t v_id) override;
        bool collapsing_check_vertex(size_t v1_id, size_t v2_id) override;
        bool swapping_check_vertex(size_t v1_id, size_t v2_id) override;
        bool smoothing_check_vertex(size_t v_id) override;

        void splitting_update_vertex(size_t v1_id, size_t v2_id, size_t v_id) override;
        void collapsing_update_vertex(size_t v1_id, size_t v2_id) override;
        void swapping_update_vertex(size_t v1_id, size_t v2_id) override;
        void smoothing_update_vertex(size_t v_id) override;
    };

    class TetAttr: BaseTetAttr{
    public:
        std::vector<Vertex> m_vertices;
        std::vector<Tet> m_tets;
        Parameters& m_params;
        Envelope& m_envelope;

        TetAttr(std::vector<Vertex>& _vertices, std::vector<Tet>& _tets, Parameters& _param, Envelope& _envelope):
                m_vertices(_vertices), m_tets(_tets), m_params(_param), m_envelope(_envelope){}
        ~TetAttr(){}

        std::vector<std::array<char, 4>> m_is_surface_fs;
        std::vector<std::array<char, 4>> m_is_bbox_fs;
        std::vector<std::array<int, 4>> m_opp_t_ids;
        std::vector<std::array<char, 4>> m_surface_tags;

        std::vector<Scalar> m_qualities;
        std::vector<Scalar> m_scalars;
        std::vector<bool> m_is_outside;

        void splitting_check_tet(size_t v1_id, size_t v2_id, size_t v_id) override;
        void collapsing_check_tet(size_t v1_id, size_t v2_id) override;
        void swapping_check_tet(size_t v1_id, size_t v2_id) override;
        void smoothing_check_tet(size_t v_id) override;

        void splitting_update_tet(size_t v1_id, size_t v2_id, size_t v_id) override;
        void collapsing_update_tet(size_t v1_id, size_t v2_id) override;
        void swapping_update_tet(size_t v1_id, size_t v2_id) override;
        void smoothing_update_tet(size_t v_id) override;
    };