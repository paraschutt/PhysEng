#pragma once
#include <vector>
#include <cstdint>
#include <cassert>

namespace apc {

// Deterministic sparse set.
// Iteration order matches insertion order. No pointer stability issues like std::map.
template<typename TEntityId, typename TValue>
class SlotMap {
public:
    TValue& insert(TEntityId id, const TValue& value) {
        assert(!contains(id));
        size_t dense_idx = dense.size();
        
        sparse[id] = dense_idx;
        dense.push_back(value);
        entities.push_back(id);
        
        return dense.back();
    }

    void erase(TEntityId id) {
        assert(contains(id));
        size_t idx_to_remove = sparse[id];
        size_t last_idx = dense.size() - 1;

        if (idx_to_remove != last_idx) {
            // Swap with last element
            TEntityId last_id = entities[last_idx];
            dense[idx_to_remove] = dense[last_idx];
            entities[idx_to_remove] = last_id;
            sparse[last_id] = idx_to_remove;
        }

        dense.pop_back();
        entities.pop_back();
        sparse.erase(id);
    }

    bool contains(TEntityId id) const {
        auto it = sparse.find(id);
        return it != sparse.end() && it->second < dense.size() && entities[it->second] == id;
    }

    TValue* get(TEntityId id) {
        if (!contains(id)) return nullptr;
        return &dense[sparse[id]];
    }

    // Iterates in strictly insertion-sorted order
    const std::vector<TEntityId>& get_ids() const { return entities; }
    const std::vector<TValue>& get_values() const { return dense; }
    size_t size() const { return dense.size(); }

private:
    // Note: FlatMap used here to ensure sparse lookup is deterministic if we ever 
    // need to iterate it. std::unordered_map is BANNED.
    FlatMap<TEntityId, size_t> sparse;
    std::vector<TValue> dense;
    std::vector<TEntityId> entities;
};

} // namespace apc