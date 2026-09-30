// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/time/Timestamp.hpp"

#include <string>
#include <string_view>

namespace navkit::app_support
{

/**
 * \brief Target-neutral lifecycle used by the mission application loop.
 *
 * \details Concrete adapters own target-specific preparation and publication details.
 * SWIL may synthesize truth and emulator data before a planned deadline, while HWIL will
 * eventually receive external transport data at that deadline. The application depends
 * only on this ordered lifecycle and never reaches into target-specific source state.
 *
 * The interface is templated only on the selected product's concrete Navigator and Logger
 * types. Runtime polymorphism is intentionally shallow and confined to desktop/application
 * support; embedded-facing NavKit algorithms remain compile-time composed.
 */
template<typename Navigator, typename Logger>
class MissionAdapter
{
public:
    virtual ~MissionAdapter() = default;

    /** Returns the first planned mission timestamp. */
    [[nodiscard]] virtual core::Timestamp t_start() const = 0;

    /** Returns true once the target-specific mission source has completed. */
    [[nodiscard]] virtual bool is_complete() const = 0;

    /**
     * \brief Initializes target-owned data flow and the Navigator at the mission start time.
     *
     * \details Called exactly once after the concrete Navigator and Logger exist. The adapter
     * owns target-specific initial data acquisition and baseline Navigator initialization, but
     * it does not run a Navigator update or replace `MissionRuntime` as the owner of phase
     * transition legality and phase-specific Navigation settings.
     */
    [[nodiscard]] virtual bool initialize(Navigator& navigator, Logger& logger) = 0;

    /**
     * \brief Prepares data that may be produced before the planned publication deadline.
     *
     * \details Called once for a planned timestamp before the application waits for that
     * deadline. A successful prepare must be followed by `publish_at_deadline()` for the same
     * timestamp before another timestamp is prepared. The adapter returns the actual prepared
     * timestamp, which may differ from the request when a finite source ends between application
     * epochs or preparation discovers a terminal source event at its native sample time.
     */
    [[nodiscard]] virtual bool prepare_before_deadline(const core::Timestamp& requested_t,
                                                       core::Timestamp& prepared_t) = 0;

    /**
     * \brief Publishes data that becomes valid at the planned deadline.
     *
     * \details The application calls this only after its Clock reaches the timestamp prepared by
     * `prepare_before_deadline()`. Publication does not run the Navigator; the application owns
     * that update explicitly so ordering remains visible and target independent.
     */
    [[nodiscard]] virtual bool
    publish_at_deadline(const core::Timestamp& t, Navigator& navigator, Logger& logger) = 0;

    /**
     * \brief Supplies post-navigation feedback needed by the selected execution target.
     *
     * \details Called after the application successfully updates the Navigator for the published
     * timestamp. SWIL uses this seam to return the selected truth-or-navigation control state to
     * its synthetic plant; HWIL may use it for target-specific feedback transport.
     */
    [[nodiscard]] virtual bool after_navigation_update(const core::Timestamp& t,
                                                       const Navigator& navigator) = 0;

    /** Returns the active stable mission-phase ID reported by the target adapter. */
    [[nodiscard]] virtual bool active_mission_phase_id(std::string& phase_id) const = 0;

    /**
     * \brief Finalizes target-owned resources after success or a partially completed lifecycle.
     *
     * \details Cleanup is best effort and idempotent from the host's perspective. Concrete
     * adapters must tolerate finalization after initialization or any later lifecycle stage.
     */
    [[nodiscard]] virtual bool finalize() = 0;

    /** Returns a stable diagnostic for the most recent adapter failure. */
    [[nodiscard]] virtual std::string_view last_error() const = 0;
};

} // namespace navkit::app_support
