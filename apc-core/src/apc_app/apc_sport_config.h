#pragma once
// =============================================================================
// apc_sport_config.h — Sport module configuration for AI utility injection
// =============================================================================
//
// Provides the data payload to initialize a full match and drive the
// sport-specific AI action injection in SceneManager:
//
//   - SportModuleType: high-level sport identifier for AI module selection
//   - SportModuleConfig: data payload for initializing match + AI rules
//
// Phase 11b Architecture:
//   When the engine boots a sport, the SceneManager calls
//   load_sport_configuration() which reads the SportModuleType and
//   clears + re-injects only the UtilityAI actions relevant to that sport.
//
// Design:
//   - Header-only, apc:: namespace
//   - No dynamic allocation
//   - Deterministic
//   - C++17
//
// =============================================================================

#include "apc_sport/apc_sport_rules.h"
#include <cstdint>

namespace apc {

// =============================================================================
// SportModuleType — High-level sport identifier for AI module loading
// =============================================================================
// Used by SceneManager::load_sport_configuration() to determine which
// subset of UtilityAI actions to inject.
//
// Separate from SportType enums in apc_sport_field.h and apc_scene_manager.h
// to avoid name collisions. Maps 1:1 to scene manager SportType values.
// =============================================================================
enum class SportModuleType : uint8_t {
    SOCCER,
    BASKETBALL,
    HOCKEY,
    AMERICAN_FOOTBALL,
    RUGBY
};

// =============================================================================
// SportModuleConfig — Data payload to initialize a full match
// =============================================================================
// Bridges the semantic rules engine (SportRulesConfig) with the AI system.
// In a full production environment, this would also include Field loading
// data (zone extents) and Physics data (ball shape, gravity, restitution).
// =============================================================================
struct SportModuleConfig {
    SportModuleType   module_type = SportModuleType::SOCCER;
    SportRulesConfig  rules;
};

} // namespace apc
