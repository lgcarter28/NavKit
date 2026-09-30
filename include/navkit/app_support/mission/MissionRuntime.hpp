// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/NavKitProductConfigPolicy.hpp"
#include "navkit/app_support/navigation/NavigationPhase.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace navkit::app_support
{

/**
 * \brief One source-independent phase in the resolved mission graph.
 *
 * \details Stable string IDs are retained for diagnostics and external adapters, while allowed
 * transitions are resolved once to checked vector indices. The Navigation selection is the only
 * phase-owned behavior applied by this runtime; guidance, plant, and synthetic-truth behavior
 * remain responsibilities of their execution adapters.
 */
struct MissionPhase
{
    std::string id{};
    NavigationPhase navigation{};
    std::vector<std::size_t> allowed_transition_indices{};
    bool terminal{false};
};

/**
 * \brief Owns mission phase identity, transition legality, and active-phase state.
 *
 * \details `MissionRuntime` is independent of SWIL truth and trajectory implementations. A
 * source or execution adapter reports its stable phase ID; this owner resolves the ID and
 * verifies that it is the configured initial phase or a declared outgoing transition. The
 * destination Navigation phase is applied before the active phase is committed, so a failed
 * application never advances the mission runtime's authoritative state.
 */
class MissionRuntime
{
public:
    MissionRuntime() = default;

    MissionRuntime(std::vector<MissionPhase> phases, const std::size_t initial_phase_index)
        : m_phases{std::move(phases)}
        , m_initial_phase_index{initial_phase_index}
    {
        index_phase_ids();
    }

    /** \brief Reports whether the resolved mission graph is structurally valid. */
    [[nodiscard]] bool is_valid() const
    {
        if (m_phases.empty() || m_initial_phase_index >= m_phases.size() ||
            m_phase_indices.size() != m_phases.size()) {
            return false;
        }

        for (const MissionPhase& phase : m_phases) {
            if (phase.id.empty() || phase.navigation.id != phase.id ||
                phase.terminal == !phase.allowed_transition_indices.empty()) {
                return false;
            }
            for (const std::size_t target_index : phase.allowed_transition_indices) {
                if (target_index >= m_phases.size()) {
                    return false;
                }
            }
        }
        return true;
    }

    /** \brief Reports whether an initial phase has been applied successfully. */
    [[nodiscard]] bool initialized() const
    {
        return m_active_phase_index.has_value();
    }

    /** \brief Returns the configured initial phase index. */
    [[nodiscard]] std::size_t initial_phase_index() const
    {
        return m_initial_phase_index;
    }

    /** \brief Returns the configured stable initial phase ID. */
    [[nodiscard]] std::string_view initial_phase_id() const
    {
        return m_initial_phase_index < m_phases.size() ? m_phases.at(m_initial_phase_index).id
                                                       : std::string_view{};
    }

    /** \brief Returns the active phase index when the runtime has been initialized. */
    [[nodiscard]] const std::optional<std::size_t>& active_phase_index() const
    {
        return m_active_phase_index;
    }

    /** \brief Returns the stable active phase ID when the runtime has been initialized. */
    [[nodiscard]] std::optional<std::string_view> active_phase_id() const
    {
        if (!m_active_phase_index.has_value() || *m_active_phase_index >= m_phases.size()) {
            return std::nullopt;
        }
        return m_phases.at(*m_active_phase_index).id;
    }

    /** \brief Reports whether the initialized runtime is currently in a terminal phase. */
    [[nodiscard]] bool active_phase_is_terminal() const
    {
        return m_active_phase_index.has_value() && *m_active_phase_index < m_phases.size() &&
               m_phases.at(*m_active_phase_index).terminal;
    }

    /** \brief Returns the number of resolved mission phases. */
    [[nodiscard]] std::size_t phase_count() const
    {
        return m_phases.size();
    }

    /** \brief Looks up a stable mission phase ID without exposing the internal lookup table. */
    [[nodiscard]] bool phase_index(const std::string_view id, std::size_t& index) const
    {
        const std::unordered_map<std::string, std::size_t>::const_iterator iter =
            m_phase_indices.find(std::string{id});
        if (iter == m_phase_indices.end()) {
            return false;
        }
        index = iter->second;
        return true;
    }

    /**
     * \brief Applies and commits the configured initial mission phase.
     *
     * \details The execution adapter's stable initial phase ID must agree with the mission graph.
     * Repeated initialization is rejected; callers use `synchronize()` after startup.
     */
    template<navkit::api::config::NavKitProductConfigPolicy NavKit>
    [[nodiscard]] bool initialize(const std::string_view reported_phase_id,
                                  typename NavKit::Navigator& navigator)
    {
        std::size_t reported_phase_index{};
        if (!phase_index(reported_phase_id, reported_phase_index)) {
            return false;
        }
        if (!is_valid() || initialized() || reported_phase_index != m_initial_phase_index) {
            return false;
        }
        if (!apply_navigation_phase<NavKit>(m_phases.at(reported_phase_index).navigation,
                                            navigator)) {
            return false;
        }
        m_active_phase_index = reported_phase_index;
        return true;
    }

    /**
     * \brief Synchronizes the active mission phase with a source-reported stable phase ID.
     *
     * \details Reporting the current phase is an idempotent no-op. A different phase must be a
     * declared outgoing transition from the active nonterminal phase. Navigation configuration is
     * applied before the active index changes, preserving the previous phase on failure.
     */
    template<navkit::api::config::NavKitProductConfigPolicy NavKit>
    [[nodiscard]] bool synchronize(const std::string_view reported_phase_id,
                                   typename NavKit::Navigator& navigator)
    {
        std::size_t reported_phase_index{};
        if (!phase_index(reported_phase_id, reported_phase_index)) {
            return false;
        }
        if (!is_valid() || !m_active_phase_index.has_value() ||
            reported_phase_index >= m_phases.size()) {
            return false;
        }

        const std::size_t active_index = *m_active_phase_index;
        if (reported_phase_index == active_index) {
            return true;
        }

        const MissionPhase& active_phase = m_phases.at(active_index);
        if (active_phase.terminal || !transition_is_allowed(active_phase, reported_phase_index)) {
            return false;
        }
        if (!apply_navigation_phase<NavKit>(m_phases.at(reported_phase_index).navigation,
                                            navigator)) {
            return false;
        }
        m_active_phase_index = reported_phase_index;
        return true;
    }

private:
    void index_phase_ids()
    {
        m_phase_indices.clear();
        m_phase_indices.reserve(m_phases.size());
        for (std::size_t index = 0U; index < m_phases.size(); ++index) {
            const MissionPhase& phase = m_phases.at(index);
            if (phase.id.empty() || !m_phase_indices.emplace(phase.id, index).second) {
                m_phase_indices.clear();
                return;
            }
        }
    }

    [[nodiscard]] static bool transition_is_allowed(const MissionPhase& phase,
                                                    const std::size_t target_index)
    {
        for (const std::size_t allowed_index : phase.allowed_transition_indices) {
            if (allowed_index == target_index) {
                return true;
            }
        }
        return false;
    }

    std::vector<MissionPhase> m_phases{};
    std::unordered_map<std::string, std::size_t> m_phase_indices{};
    std::size_t m_initial_phase_index{};
    std::optional<std::size_t> m_active_phase_index{};
};

} // namespace navkit::app_support
