// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/state/State.hpp"
#include "navkit/io/CsvWriter.hpp"
#include "navkit/io/log_payloads/NavEstimateLogPayload.hpp"

#include <cmath>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <string_view>

namespace navkit::io
{

class NavEstimateLogProduct
{
public:
    static constexpr std::string_view LogKey = "nav";

    void open(const std::filesystem::path& output_dir)
    {
        m_csv.open(output_dir / "nav.csv",
                   {"time_s",
                    "p_e_x_m",
                    "p_e_y_m",
                    "p_e_z_m",
                    "err_p_e_x_m",
                    "err_p_e_y_m",
                    "err_p_e_z_m",
                    "sigma_p_e_x_m",
                    "sigma_p_e_y_m",
                    "sigma_p_e_z_m"});
    }

    template<typename StateDef, typename Filter>
    void log(const NavEstimateLogPayload<StateDef, Filter>& payload)
    {
        using Nominal = typename StateDef::Nominal;
        using Error = typename StateDef::Error;

        const auto p_est = payload.filter.state().template segment<3>(Nominal::Pos::i);
        const auto err = p_est - payload.truth.p_e;
        const auto& P = payload.filter.covariance();

        m_csv.write_row(payload.time_s,
                        p_est.x(),
                        p_est.y(),
                        p_est.z(),
                        err.x(),
                        err.y(),
                        err.z(),
                        std::sqrt(P(Error::Pos::i + 0, Error::Pos::i + 0)),
                        std::sqrt(P(Error::Pos::i + 1, Error::Pos::i + 1)),
                        std::sqrt(P(Error::Pos::i + 2, Error::Pos::i + 2)));
    }

    void flush()
    {
        m_csv.flush();
    }

    static nlohmann::json metadata()
    {
        return {{"schema", "nav_estimate_v1"},
                {"file", "nav.csv"},
                {"units", {{"time", "s"}, {"p_e", "m"}, {"err_p_e", "m"}, {"sigma_p_e", "m"}}}};
    }

    static nlohmann::json manifest_entry()
    {
        return {{"csv", "nav.csv"}, {"manifest", "nav.meta.json"}};
    }

private:
    CsvWriter m_csv;
};

} // namespace navkit::io
