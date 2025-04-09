template <typename T>
class Attribute
{
public:
    using MapResult = internal::MapResult<T, Eigen::Dynamic>;
    using ConstMapResult = internal::ConstMapResult<T, Eigen::Dynamic>;

    template <typename U, int D>
    friend class AccessorBase;
    friend class internal::AttributeTransactionStack<T>;

    void serialize(const std::string& name, const int dim, MeshWriter& writer) const;

    Attribute(const std::string& name, int64_t dimension, T default_value = T(0), int64_t size = 0);

    Attribute(Attribute&& o);
    ~Attribute();
    Attribute& operator=(Attribute&& o);

    ConstMapResult vector_attribute(const int64_t index) const;
    MapResult vector_attribute(const int64_t index);

    T scalar_attribute(const int64_t index) const;
    T& scalar_attribute(const int64_t index);

    int64_t reserved_size() const;

    int64_t dimension() const;
    void reserve(const int64_t size);

    const T& default_value() const;

    bool operator==(const Attribute<T>& o) const;

    void push_scope();
    void pop_scope(bool apply_updates);
    void rollback_current_scope();

    const AttributeTransactionStack<T>& get_local_scope_stack() const;
    AttributeTransactionStack<T>& get_local_scope_stack();

    void consolidate(const std::vector<int64_t>& new2old);

    void index_remap(const std::vector<T>& old2new);
    void index_remap(const std::vector<T>& old2new, const std::vector<Eigen::Index>& cols);


    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////
    // new stuff

    std::string name() const;

    struct SplitStrategies
    {
        SplitFuncType split_spine_strategy;
        SplitRibFuncType split_rib_strategy;
    };

    void set_split_strategies(const SplitStrategies& strategies);
    const SplitStrategies& get_split_strategies() const;

    struct CollapseStrategies
    {
        CollapseFuncType collapse_strategy;
    };

    void set_collapse_strategies(const CollapseStrategies& strategies);
    const CollapseStrategies& get_collapse_strategies() const;

    struct SwapStrategies
    {
        SplitFuncType split_spine_strategy;
        SplitRibFuncType split_rib_strategy;
        CollapseFuncType collapse_strategy;
    };

    void set_swap_strategies(const SwapStrategies& strategies);
    const SwapStrategies& get_swap_strategies() const;

    struct FaceSplitStrategies
    {
        SplitFuncType split_1_spine_strategy;
        SplitRibFuncType split_1_rib_strategy;
        SplitFuncType split_2_spine_strategy;
        SplitRibFuncType split_2_rib_strategy;
        CollapseFuncType collapse_strategy;
    };

    void set_face_split_strategies(const FaceSplitStrategies& strategies);
    const FaceSplitStrategies& get_face_split_strategies() const;

    // set strategies to make it behave like a position
    void set_default_position_strategies();
    // set strategies to make it behave like a tag
    void set_default_tag_strategies();
    // set none strategies
    void set_none_strategies();

    // check if all strategies were set and print info
    bool validate_strategies(const spdlog::log_level& l = spdlog::log_level::NONE);


private:
    std::vector<T> m_data;
    PerThreadAttributeScopeStacks<T> m_scope_stacks;
    int64_t m_dimension = -1;
    T m_default_value = T(0);
    std::string m_name;

    SplitStrategies m_split_strategies;
    CollapseStrategies m_collapse_strategies;
    SwapStrategies m_swap_strategies;
    FaceSplitStrategies m_face_split_strategies;
};