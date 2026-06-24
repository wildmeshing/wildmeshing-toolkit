#pragma once

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/Logger.hpp>

// #include <tbb/concurrent_vector.h>
// #include <tbb/enumerable_thread_specific.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <vector>

namespace wmtk {
/**
 * @brief serving as buffers for attributes data that can be modified by operations
 *
 */
class AbstractAttributeContainer
{
public:
    virtual ~AbstractAttributeContainer() = default;
    virtual void move(size_t from, size_t to) {};
    virtual void resize(size_t) = 0;
    virtual void clear() = 0;
    virtual void rollback() = 0;
    virtual void begin_protect() = 0;
    virtual void end_protect() = 0;
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
        // if (m_attributes.size() > s) {
        //     m_attributes.resize(s);
        //     m_attributes.shrink_to_fit();
        // }
        // TODO: in Concurrent, vertex partition id, vertex mutex should be part of attribute

        m_attributes.resize(s);
    }
    void clear() override { m_attributes.clear(); }

    bool assign(size_t to, T&& val) // always use this in OP_after
    {
        // m_attributes[to] = val;
        // if (recording.local()) m_rollback_list.local()[to] = val;
        // // TODO: are locks necessary? not now.
        // return true;

        m_attributes[to] = val;
        std::thread::id tid = std::this_thread::get_id();
        try_init_local_storage(tid);
        if (recording_map[tid]) m_rollback_list_map[tid][to] = val;

        return true;
    }
    /**
     * @brief retrieve the protected attribute data on operation-fail
     *
     */
    void rollback() override
    {
        // for (auto& [i, v] : m_rollback_list.local()) {
        //     m_attributes[i] = std::move(v);
        // }
        for (auto& [i, v] : m_rollback_list_map[std::this_thread::get_id()]) {
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
        std::thread::id tid = std::this_thread::get_id();
        try_init_local_storage(tid);
        m_rollback_list_map[tid].clear();
        recording_map[tid] = true;
    };
    /**
     * @brief clear local buffers and finish recording
     *
     */
    void end_protect() override
    {
        std::thread::id tid = std::this_thread::get_id();
        m_rollback_list_map[tid].clear();
        recording_map[tid] = false;
    }

    const T& at(size_t i) const { return m_attributes[i]; }

    const T& operator[](size_t i) const { return at(i); }

    T& operator[](size_t i)
    {
        std::thread::id tid = std::this_thread::get_id();
        try_init_local_storage(tid);
        if (recording_map[tid]) {
            m_rollback_list_map[tid].emplace(i, m_attributes[i]);
        }
        return m_attributes[i];
    }


    size_t size() const { return m_attributes.size(); }
    // tbb::enumerable_thread_specific<std::unordered_map<size_t, T>> m_rollback_list;
    // // experimenting with tbb, could be templated as well.
    // tbb::concurrent_vector<T> m_attributes;
    // tbb::enumerable_thread_specific<bool> recording{false};

    void try_init_local_storage(const std::thread::id& tid)
    {
        // if not initialize, initialize
        // TODO: this brings extra cost, better way?
        if (recording_map.find(tid) == recording_map.end()) {
            recording_map[tid] = false;
        }
        if (m_rollback_list_map.find(tid) == m_rollback_list_map.end()) {
            m_rollback_list_map[tid] = std::unordered_map<size_t, T>();
        }
    }

    std::map<std::thread::id, std::unordered_map<size_t, T>> m_rollback_list_map;
    std::map<std::thread::id, bool> recording_map;
    std::vector<T> m_attributes;
};
} // namespace wmtk
