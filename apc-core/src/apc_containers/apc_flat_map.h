#pragma once
#include <vector>
#include <algorithm>
#include <cstdint>

namespace apc {

// A sorted vector of key-value pairs. 
// Iteration order is STRICTLY alphabetical/numerical by Key.
// O(N) insert, O(log N) lookup. Perfect for data that changes rarely.
template<typename TKey, typename TValue>
class FlatMap {
public:
    struct Pair {
        TKey key;
        TValue value;
        
        // Strict weak ordering for determinism
        bool operator<(const Pair& other) const { return key < other.key; }
        bool operator==(const Pair& other) const { return key == other.key; }
    };

    // Returns pointer to value, or nullptr if not found
    TValue* find(const TKey& key) {
        Pair search{};
        search.key = key;
        auto it = std::lower_bound(pairs.begin(), pairs.end(), search);
        if (it != pairs.end() && it->key == key) {
            return &it->value;
        }
        return nullptr;
    }

    const TValue* find(const TKey& key) const {
        Pair search{};
        search.key = key;
        auto it = std::lower_bound(pairs.begin(), pairs.end(), search);
        if (it != pairs.end() && it->key == key) {
            return &it->value;
        }
        return nullptr;
    }

    // Insert or update. Maintains sort order.
    void insert(const TKey& key, const TValue& value) {
        auto existing = find(key);
        if (existing) {
            *existing = value;
            return;
        }
        Pair p{key, value};
        pairs.insert(std::upper_bound(pairs.begin(), pairs.end(), p), p);
    }

    size_t size() const { return pairs.size(); }
    bool empty() const { return pairs.empty(); }
    
    // Deterministic iteration
    typename std::vector<Pair>::iterator begin() { return pairs.begin(); }
    typename std::vector<Pair>::iterator end() { return pairs.end(); }
    typename std::vector<Pair>::const_iterator begin() const { return pairs.begin(); }
    typename std::vector<Pair>::const_iterator end() const { return pairs.end(); }

private:
    std::vector<Pair> pairs;
};

} // namespace apc