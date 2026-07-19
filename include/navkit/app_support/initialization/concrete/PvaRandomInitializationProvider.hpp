// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/initialization/NavInitialization.hpp"
#include "navkit/app_support/initialization/PvaInitializationJson.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"

#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>

namespace navkit::app_support
{

struct PvaRandomInitializationProvider
{
    static constexpr const char* runtime_type = "pva_random_error";

    static void validate_runtime_config(const nlohmann::json& cfg)
    {
        const nlohmann::json& initialization = detail::require_object(cfg, "pva_initialization");
        detail::require_pva_initialization_type(initialization, runtime_type);
        detail::validate_pva_error_covariance_shape(initialization);
        const std::string pva_error_frame = detail::pva_error_frame_from_json(initialization);
        if (pva_error_frame.empty()) {
            detail::throw_runtime_config_error(
                "pva_initialization.pva_error_frame must not be empty");
        }
        detail::require_unsigned_integer(initialization, "seed");
    }

    [[nodiscard]] static PvaInitialization initialize(const nlohmann::json& cfg,
                                                      const TrajectoryRun& trajectory)
    {
        const nlohmann::json& initialization = cfg.at("pva_initialization");
        PvaInitialization pva_init = detail::base_pva_initialization(trajectory);
        const core::Vec3 reference_p_e_m = core::estimation::pos_e_m(pva_init.pva);
        const core::estimation::PvaCovariance covariance =
            detail::pva_error_covariance_from_json(initialization, reference_p_e_m);
        const std::uint64_t seed = initialization.at("seed").get<std::uint64_t>();

        detail::apply_pva_error(pva_init, detail::sample_pva_error(covariance, seed));
        return pva_init;
    }
};

} // namespace navkit::app_support
