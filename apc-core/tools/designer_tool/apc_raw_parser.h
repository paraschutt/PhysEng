#pragma once
#include <string>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include "../../src/apc_math/apc_fixed.h"

namespace apc::tools {

// Reads raw text configuration files directly to avoid intermediate JSON
// converter failures. Standard IDE extensions or GUI tools that convert
// text files into JSON often lack converters for custom data types like
// FixedPoint, leading to silent failures and dropped precision.
//
// File format (example):
//   # Comment lines start with '#'
//   friction = 0.75
//   ball_radius = 0.11
//   max_speed = 8.5
//
// Interfacing directly with raw .txt files guarantees the exact authored
// values are ingested into the engine.
class RawConfigParser {
private:
    std::unordered_map<std::string, std::string> key_value_store;

public:
    bool load_from_text(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) return false;

        std::string line;
        while (std::getline(file, line)) {
            // Basic raw text parsing: ignore comments and empty lines
            if (line.empty() || line[0] == '#') continue;

            std::istringstream iss(line);
            std::string key, value;
            if (std::getline(iss, key, '=') && std::getline(iss, value)) {
                key_value_store[trim(key)] = trim(value);
            }
        }
        return true;
    }

    FixedPoint get_fixedpoint(const std::string& key, FixedPoint default_val = FixedPoint(0)) {
        auto it = key_value_store.find(key);
        if (it != key_value_store.end()) {
            // Parse directly from raw text float representation to deterministic FixedPoint
            float raw_float = std::stof(it->second);
            return FixedPoint(raw_float);
        }
        return default_val;
    }

    float get_float(const std::string& key, float default_val = 0.0f) {
        auto it = key_value_store.find(key);
        if (it != key_value_store.end()) {
            return std::stof(it->second);
        }
        return default_val;
    }

    int32_t get_int(const std::string& key, int32_t default_val = 0) {
        auto it = key_value_store.find(key);
        if (it != key_value_store.end()) {
            return std::stoi(it->second);
        }
        return default_val;
    }

    bool has_key(const std::string& key) const {
        return key_value_store.find(key) != key_value_store.end();
    }

    uint32_t key_count() const {
        return static_cast<uint32_t>(key_value_store.size());
    }

private:
    std::string trim(const std::string& str) {
        size_t first = str.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) return "";
        size_t last = str.find_last_not_of(" \t\r\n");
        return str.substr(first, (last - first + 1));
    }
};

} // namespace apc::tools
