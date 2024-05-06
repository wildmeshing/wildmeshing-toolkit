#include <igl/Timer.h>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <map>

namespace wmtk {
/**
 * @brief serving as buffers for attributes data that can be modified by operations
 *
 */
class AbstractAttributeContainer
{
public:
    virtual ~AbstractAttributeContainer() = default;
    virtual void move(size_t from, size_t to){};
    virtual void resize(size_t){};
    virtual void rollback(){};
    virtual void begin_protect(){};
    virtual void end_protect(){};
};


template <typename T>
struct AttributeCollection : public AbstractAttributeContainer
{
    void move(size_t from, size_t to) override
    {
        if (from == to) return;
        m_attributes[to] = std::move(m_attributes[from]);
    }
    void resize(size_t s) override
    {
        // m_attributes.grow_to_at_least(s);
        m_attributes.resize(s);
        // if (m_attributes.size() > s) {
        //     m_attributes.resize(s);
        //     m_attributes.shrink_to_fit();
        // }
        // TODO: in Concurrent, vertex partition id, vertex mutex should be part of attribute
    }

    bool assign(size_t to, T&& val) // always use this in OP_after
    {
        m_attributes[to] = val;
        if (recording) m_rollback_list[to] = val;
        // TODO: are locks necessary? not now.
        return true;
    }
    /**
     * @brief retrieve the protected attribute data on operation-fail
     *
     */
    void rollback() override
    {
        for (auto& [i, v] : m_rollback_list) {
            m_attributes[i] = std::move(v);
        }
        end_protect();
    }
    /**
     * @brief clean local buffers for attribute, and start recording
     *
     */
    void begin_protect() override
    {
        m_rollback_list.clear();
        recording = true;
    };
    /**
     * @brief clear local buffers and finish recording
     *
     */
    void end_protect() override
    {
        m_rollback_list.clear();
        recording = false;
    }

    const T& operator[](size_t i) const { return m_attributes[i]; }

    T& operator[](size_t i)
    {
        if (recording) {
            m_rollback_list.emplace(i, m_attributes[i]);
            // m_rollback_list_pair.emplace_back(i, m_attributes[i]);
        }
        return m_attributes[i];
    }

    const T& at(size_t i) const { return m_attributes[i]; }

    size_t size() const { return m_attributes.size(); }
    std::map<size_t, T> m_rollback_list;
    // experimenting with tbb, could be templated as well.
    std::vector<T> m_attributes;
    bool recording = false;
    std::vector<std::pair<size_t, T>> m_rollback_list_pair;
};

template <typename T>
struct AttributeCollectionWithVectorPair : public AbstractAttributeContainer
{
    void move(size_t from, size_t to) override
    {
        if (from == to) return;
        m_attributes[to] = std::move(m_attributes[from]);
    }
    void resize(size_t s) override
    {
        // m_attributes.grow_to_at_least(s);
        m_attributes.resize(s);
        // if (m_attributes.size() > s) {
        //     m_attributes.resize(s);
        //     m_attributes.shrink_to_fit();
        // }
        // TODO: in Concurrent, vertex partition id, vertex mutex should be part of attribute
    }

    bool assign(size_t to, T&& val) // always use this in OP_after
    {
        m_attributes[to] = val;
        if (recording) m_rollback_list[to] = val;
        // TODO: are locks necessary? not now.
        return true;
    }
    /**
     * @brief retrieve the protected attribute data on operation-fail
     *
     */
    void rollback() override
    {
        for (auto& [i, v] : m_rollback_list) {
            m_attributes[i] = std::move(v);
        }
        end_protect();
    }
    /**
     * @brief clean local buffers for attribute, and start recording
     *
     */
    void begin_protect() override
    {
        m_rollback_list.clear();
        recording = true;
    };
    /**
     * @brief clear local buffers and finish recording
     *
     */
    void end_protect() override
    {
        m_rollback_list.clear();
        recording = false;
    }

    const T& operator[](size_t i) const { return m_attributes[i]; }

    T& operator[](size_t i)
    {
        if (recording) {
            // m_rollback_list.emplace(i, m_attributes[i]);
            m_rollback_list_pair.emplace_back(i, m_attributes[i]);
        }
        return m_attributes[i];
    }

    const T& at(size_t i) const { return m_attributes[i]; }

    size_t size() const { return m_attributes.size(); }
    std::map<size_t, T> m_rollback_list;
    // experimenting with tbb, could be templated as well.
    std::vector<T> m_attributes;
    bool recording = false;
    std::vector<std::pair<size_t, T>> m_rollback_list_pair;
};

template <typename T>
struct AttributeCollectionWithVectorPairStatic : public AbstractAttributeContainer
{
    void move(size_t from, size_t to) override
    {
        if (from == to) return;
        m_attributes[to] = std::move(m_attributes[from]);
    }
    void resize(size_t s) override
    {
        // m_attributes.grow_to_at_least(s);
        m_attributes.resize(s);
        // if (m_attributes.size() > s) {
        //     m_attributes.resize(s);
        //     m_attributes.shrink_to_fit();
        // }
        // TODO: in Concurrent, vertex partition id, vertex mutex should be part of attribute
    }

    bool assign(size_t to, T&& val) // always use this in OP_after
    {
        m_attributes[to] = val;
        if (recording) m_rollback_list[to] = val;
        // TODO: are locks necessary? not now.
        return true;
    }
    /**
     * @brief retrieve the protected attribute data on operation-fail
     *
     */
    void rollback() override
    {
        for (auto& [i, v] : m_rollback_list) {
            m_attributes[i] = std::move(v);
        }
        end_protect();
    }
    /**
     * @brief clean local buffers for attribute, and start recording
     *
     */
    void begin_protect() override
    {
        m_rollback_list.clear();
        recording = true;
    };
    /**
     * @brief clear local buffers and finish recording
     *
     */
    void end_protect() override
    {
        m_rollback_list.clear();
        recording = false;
    }

    const T& operator[](size_t i) const { return m_attributes[i]; }

    T& operator[](size_t i)
    {
        if (recording) {
            // m_rollback_list.emplace(i, m_attributes[i]);
            // m_rollback_list_pair.emplace_back(i, m_attributes[i]);
            // m_rollback_list_pair.push_back(std::make_pair(i, m_attributes[i]));
            push_back(std::make_pair(i, m_attributes[i]));
        }
        return m_attributes[i];
    }

    static void push_back(std::pair<size_t, T> value) { m_rollback_list_pair.push_back(value); }

    const T& at(size_t i) const { return m_attributes[i]; }

    size_t size() const { return m_attributes.size(); }
    std::map<size_t, T> m_rollback_list;
    std::vector<T> m_attributes;
    bool recording = false;
    static std::vector<std::pair<size_t, T>> m_rollback_list_pair;
};
} // namespace wmtk

TEST_CASE("attribute_performance", "[attribute][.]")
{
    using namespace wmtk;
    std::vector<int> raw_vector;
    AttributeCollection<int> attribute_collection;
    AttributeCollectionWithVectorPair<int> ac_vectorpair;
    AttributeCollectionWithVectorPairStatic<int> ac_static;

    double time = 0;
    igl::Timer timer;

    size_t N = 200000;
    size_t iter = 1000000;

    raw_vector.resize(N);
    attribute_collection.resize(N);
    ac_vectorpair.resize(N);
    ac_static.resize(N);

    timer.start();
    // for (size_t k = 0; k < iter; ++k) {
    //     for (size_t i = 0; i < N; ++i) {
    //         raw_vector[i] = i;
    //     }
    // }
    for (size_t k = 0; k < iter; ++k) {
        for (size_t i = 0; i < 40; ++i) {
            raw_vector[rand() % N] = rand();
        }
    }
    std::cout << "raw vector write: " << timer.getElapsedTimeInMilliSec() << std::endl;


    // attribute_collection.end_protect();
    timer.start();
    // for (size_t k = 0; k < iter; ++k) {
    //     for (size_t i = 0; i < N; ++i) {
    //         attribute_collection[i] = i;
    //     }
    // }
    for (size_t k = 0; k < iter; ++k) {
        attribute_collection.begin_protect();
        for (size_t i = 0; i < 40; ++i) {
            attribute_collection[rand() % N] = rand();
        }
        attribute_collection.end_protect();
    }
    std::cout << "attribute collection with map write: " << timer.getElapsedTimeInMilliSec()
              << std::endl;

    // ac_vectorpair.begin_protect();
    // attribute_collection.end_protect();
    timer.start();
    // for (size_t k = 0; k < iter; ++k) {
    //     for (size_t i = 0; i < N; ++i) {
    //         ac_vectorpair[i] = i;
    //     }
    // }
    for (size_t k = 0; k < iter; ++k) {
        ac_vectorpair.begin_protect();
        for (size_t i = 0; i < 40; ++i) {
            ac_vectorpair[rand() % N] = rand();
        }
        ac_vectorpair.end_protect();
    }
    std::cout << "attribute collection with vector pair write: " << timer.getElapsedTimeInMilliSec()
              << std::endl;

    // ac_static.begin_protect();
    // // attribute_collection.end_protect();
    // timer.start();
    // for (size_t k = 0; k < iter; ++k) {
    //     for (size_t i = 0; i < N; ++i) {
    //         ac_static[i] = 1;
    //     }
    // }
    // std::cout << "attribute collection with static vector pair write: "
    //           << timer.getElapsedTimeInMilliSec() << std::endl;
}


TEST_CASE("attribute_performance_2", "[attribute][.]")
{
    using namespace wmtk;
    std::vector<int> raw_vector;
    AttributeCollection<int> attribute_collection;
    AttributeCollectionWithVectorPair<int> ac_vectorpair;
    AttributeCollectionWithVectorPairStatic<int> ac_static;

    double time = 0;
    igl::Timer timer;

    size_t N = 20;
    size_t iter = 1000000;

    raw_vector.resize(N);
    attribute_collection.resize(N);
    ac_vectorpair.resize(N);
    ac_static.resize(N);

    timer.start();
    for (size_t k = 0; k < iter; ++k) {
        for (size_t i = 0; i < N; ++i) {
            raw_vector[i] = 1;
        }
    }
    std::cout << "raw vector write: " << timer.getElapsedTimeInMilliSec() << std::endl;


    // attribute_collection.begin_protect();
    // // attribute_collection.end_protect();
    // timer.start();
    // for (size_t k = 0; k < iter; ++k) {
    //     for (size_t i = 0; i < N; ++i) {
    //         attribute_collection[i] = 1;
    //     }
    // }
    // std::cout << "attribute collection with map write: " << timer.getElapsedTimeInMilliSec()
    //           << std::endl;

    ac_vectorpair.begin_protect();
    // attribute_collection.end_protect();
    timer.start();
    for (size_t k = 0; k < iter; ++k) {
        for (size_t i = 0; i < N; ++i) {
            ac_vectorpair[i] = 1;
        }
    }
    std::cout << "attribute collection with vector pair write: " << timer.getElapsedTimeInMilliSec()
              << std::endl;

    // ac_static.begin_protect();
    // // attribute_collection.end_protect();
    // timer.start();
    // for (size_t k = 0; k < iter; ++k) {
    //     for (size_t i = 0; i < N; ++i) {
    //         ac_static[i] = 1;
    //     }
    // }
    // std::cout << "attribute collection with static vector pair write: "
    //           << timer.getElapsedTimeInMilliSec() << std::endl;
}
