// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/io/CsvWriter.hpp"
#include "navkit/io/StateErrorLogSchema.hpp"
#include "navkit/io/log_payloads/NavEstimateLogPayload.hpp"

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string_view>

namespace navkit::io
{

template<typename StateDef, typename Filter>
class NavEstimateLogProduct
{
public:
    static constexpr std::string_view LogKey = "nav_estimate_ecef";

    void configure(const nlohmann::json& cfg)
    {
        m_covariance_mode = detail::covariance_log_mode_from_json(cfg, "nav_estimate");
    }

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "nav_estimate_ecef.csv",
                   detail::nominal_state_with_covariance_header<Nominal, Error>(m_covariance_mode));
    }

    void log(const NavEstimateLogPayload<StateDef, Filter>& payload)
    {
        m_csv.write_row_values(
            detail::nominal_state_with_covariance_row<Nominal, Error>(payload.time_s,
                                                                      payload.filter.state(),
                                                                      payload.filter.covariance(),
                                                                      m_covariance_mode));
    }

    void flush()
    {
        m_csv.flush();
    }

    nlohmann::json metadata() const
    {
        return {{"schema", "nav_estimate_v1"},
                {"file", "nav_estimate_ecef.csv"},
                {"covariance", detail::covariance_log_mode_name(m_covariance_mode)},
                {"description",
                 "Nominal ECEF navigation state estimate with matching error-state covariance."},
                {"units",
                 {{"time", "s"},
                  {"p_e", "m"},
                  {"v_e", "m/s"},
                  {"q_b2e", "unit quaternion"},
                  {"gyro_bias_b", "rad/s"},
                  {"accel_bias_b", "m/s^2"}}}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "nav_estimate_ecef.csv"}, {"manifest", "nav_estimate_ecef.meta.json"}};
    }

private:
    using Nominal = typename StateDef::Nominal;
    using Error = typename StateDef::Error;

    CsvWriter m_csv;
    detail::CovarianceLogMode m_covariance_mode{detail::CovarianceLogMode::Diagonal};
};

} // namespace navkit::io
